package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Command that allows the driver to drive with full translation control while the 
 * robot automatically rotates to face the hub.
 * 
 * <p>This is designed to be toggled on/off via a button press. When active:
 * <ul>
 *   <li>Left stick translation works exactly as normal driving (field-centric, with speed multipliers)</li>
 *   <li>Right stick rotation is OVERRIDDEN — the robot automatically aims at the hub</li>
 *   <li>The operator can fire while the driver repositions</li>
 * </ul>
 * 
 * <p>Hub aiming uses the same proven PD control from PrepareToFire:
 * <ul>
 *   <li>When Limelight sees the hub tag: uses TX directly for self-correcting visual feedback</li>
 *   <li>When hub tag is lost: LimelightSubsystem6237 provides odometry-estimated TX as fallback</li>
 *   <li>If hub has never been seen: no rotation applied (driver has full manual control)</li>
 * </ul>
 * 
 * <p>Requires the drivetrain subsystem, so it interrupts the default drive command.
 * When the command ends (toggle off), normal driving resumes automatically.
 */
public class AimAtHubWhileDriving extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    
    // Field-centric drive request — same setup as DriverController's default
    // Uses BlueAlliance perspective so CTRE handles Red alliance flip via setOperatorPerspectiveForward()
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(0)  // We handle our own rotation control
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
    // --- PD Control Gains (same proven values as PrepareToFire) ---
    private static final double PROPORTIONAL_GAIN = 0.03;   // rad/s per degree of TX error
    private static final double DERIVATIVE_GAIN = 0.005;    // Damping to reduce oscillation
    private static final double MIN_ROTATION_SPEED = 0.15;  // rad/s minimum to overcome friction
    private static final double AIM_TOLERANCE_DEGREES = 3.0; // "Aimed" when within this
    private static final double FINE_AIM_THRESHOLD = 8.0;   // Below this, don't apply min speed
    
    // State tracking
    private Rotation2d lastTargetHeading = null;
    private boolean hasEverSeenTarget = false;
    private double lastTx = 0;  // For derivative calculation

    /**
     * Creates a new AimAtHubWhileDriving command.
     * 
     * @param limelight The Limelight subsystem for hub tracking
     * @param drivetrain The swerve drivetrain
     * @param driverController The driver's Xbox controller (for translation input)
     */
    public AimAtHubWhileDriving(LimelightSubsystem6237 limelight, 
                                 CommandSwerveDrivetrain drivetrain, 
                                 CommandXboxController driverController) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        lastTargetHeading = null;
        hasEverSeenTarget = false;
        lastTx = 0;
        SmartDashboard.putBoolean("AimAtHub Mode", true);
    }

    @Override
    public void execute() {
        // === TRANSLATION: Pass through driver input exactly as default drive ===
        // Mirrors the speed multiplier logic from DriverController.mapXboxController
        double speedMultiplier = 1.0;
        boolean slowTrigger = driverController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold;
        boolean fastTrigger = driverController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold;
        
        if (slowTrigger) {
            speedMultiplier = 15.0 / 35.0;  // Scale to 15%
        } else if (fastTrigger) {
            speedMultiplier = 55.0 / 35.0;  // Scale to 55%
        }
        
        // Same inversion logic as DriverController
        double velocityX = DriverController.invertXNumberFieldCentric * driverController.getLeftY() 
                           * Constants.TempSwerve.MaxSpeed * speedMultiplier;
        double velocityY = DriverController.invertYNumberFieldCentric * driverController.getLeftX() 
                           * Constants.TempSwerve.MaxSpeed * speedMultiplier;
        
        // === ROTATION: Automatic hub aiming via Limelight TX + PD control ===
        Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
        
        // getLastHubTx() returns:
        //   - Direct Limelight TX when hub tag is visible (most accurate)
        //   - Odometry-estimated TX when hub tag is lost but was previously seen
        //   - 0.0 if hub has never been seen
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        boolean hubEverSeen = limelight.hasEverSeenHub();
        double tx = limelight.getLastHubTx();
        
        double rotationalRate = 0;
        
        if (hubVisible) {
            // === DIRECT VISION TRACKING ===
            hasEverSeenTarget = true;
            
            // Store target heading for "lost target" fallback within this command
            lastTargetHeading = currentHeading.plus(Rotation2d.fromDegrees(tx));
            
            // PD control on TX
            double txDerivative = tx - lastTx;
            lastTx = tx;
            
            if (Math.abs(tx) > AIM_TOLERANCE_DEGREES) {
                double pTerm = -tx * PROPORTIONAL_GAIN;
                double dTerm = -txDerivative * DERIVATIVE_GAIN;
                rotationalRate = pTerm + dTerm;
                
                // Only apply minimum speed outside fine aim zone
                if (Math.abs(tx) > FINE_AIM_THRESHOLD) {
                    if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                        rotationalRate = MIN_ROTATION_SPEED;
                    } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                        rotationalRate = -MIN_ROTATION_SPEED;
                    }
                }
            } else {
                lastTx = 0;
            }
            
        } else if (hubEverSeen || (hasEverSeenTarget && lastTargetHeading != null)) {
            // === ODOMETRY FALLBACK ===
            // LimelightSubsystem6237 continuously updates lastHubTx via odometry
            // when the hub isn't visible, so we can still use tx from the subsystem.
            // Additionally, if we had a target heading from within this command session,
            // use heading-based control as a secondary fallback.
            
            if (hubEverSeen) {
                // Use subsystem's odometry-estimated TX (updated every cycle)
                double txDerivative = tx - lastTx;
                lastTx = tx;
                
                if (Math.abs(tx) > AIM_TOLERANCE_DEGREES) {
                    double pTerm = -tx * PROPORTIONAL_GAIN;
                    double dTerm = -txDerivative * DERIVATIVE_GAIN;
                    rotationalRate = pTerm + dTerm;
                    
                    if (Math.abs(tx) > FINE_AIM_THRESHOLD) {
                        if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                            rotationalRate = MIN_ROTATION_SPEED;
                        } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                            rotationalRate = -MIN_ROTATION_SPEED;
                        }
                    }
                } else {
                    lastTx = 0;
                }
            } else {
                // Heading-based fallback from this command session
                Rotation2d error = lastTargetHeading.minus(currentHeading);
                double errorDegrees = error.getDegrees();
                
                if (Math.abs(errorDegrees) > AIM_TOLERANCE_DEGREES) {
                    rotationalRate = errorDegrees * PROPORTIONAL_GAIN;
                    if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                        rotationalRate = MIN_ROTATION_SPEED;
                    } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                        rotationalRate = -MIN_ROTATION_SPEED;
                    }
                }
            }
            
        } else {
            // === NEVER SEEN HUB — let driver rotate manually ===
            // Fall back to right stick rotation so the driver isn't stuck
            rotationalRate = -1 * driverController.getRightX() 
                             * Constants.TempSwerve.MaxAngularRate * speedMultiplier;
        }
        
        // Clamp rotation rate
        rotationalRate = Math.max(-Constants.TempSwerve.MaxAngularRate, 
                        Math.min(Constants.TempSwerve.MaxAngularRate, rotationalRate));
        
        // === APPLY COMBINED DRIVE REQUEST ===
        drivetrain.setControl(
            driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public boolean isFinished() {
        // Runs until toggled off
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion briefly — default drive command will resume immediately
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        SmartDashboard.putBoolean("AimAtHub Mode", false);
    }
}

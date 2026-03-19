package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

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
 * <p>Hub aiming uses PD control from Limelight TX:
 * <ul>
 *   <li>When Limelight sees the hub tag: uses TX directly for self-correcting visual feedback</li>
 *   <li>When hub tag is lost: HOLDS the last heading where hub was visible (no rotation)</li>
 *   <li>If hub has never been seen: no rotation applied (driver has full manual control)</li>
 * </ul>
 * 
 * <p>Max rotation rate is clamped to keep the mode gentle and predictable.
 * 
 * <p>Requires the drivetrain subsystem, so it interrupts the default drive command.
 * When the command ends (toggle off), normal driving resumes automatically.
 */
public class AimAtHubWhileDriving extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    
    // Field-centric drive request -- same setup as DriverController's default
    // Uses OperatorPerspective so CTRE handles Red alliance flip via setOperatorPerspectiveForward()
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(0)  // We handle our own rotation control
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    
    // --- PD Control Gains ---
    private static final double PROPORTIONAL_GAIN = 0.03;   // rad/s per degree of TX error
    private static final double DERIVATIVE_GAIN = 0.005;    // Damping to reduce oscillation
    private static final double MIN_ROTATION_SPEED = 0.15;  // rad/s minimum to overcome friction
    private static final double AIM_TOLERANCE_DEGREES = 3.0; // "Aimed" when within this
    private static final double FINE_AIM_THRESHOLD = 8.0;   // Below this, don't apply min speed
    
    // Max rotation rate for this mode — keeps things gentle and predictable
    // About 1/3 of the robot's full angular rate
    private static final double MAX_AIM_ROTATION_RATE = 1.5; // rad/s
    
    // State tracking
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
        
        // Same inversion logic as DriverController (including translation swap)
        double swapMult = DriverController.getTranslationSwapMultiplier();
        double velocityX = swapMult * DriverController.invertXNumberFieldCentric * driverController.getLeftY() 
                           * Constants.TempSwerve.MaxSpeed * speedMultiplier;
        double velocityY = swapMult * DriverController.invertYNumberFieldCentric * driverController.getLeftX() 
                           * Constants.TempSwerve.MaxSpeed * speedMultiplier;
        
        // === ROTATION: Automatic hub aiming via Limelight TX + PD control ===
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        double tx = limelight.getHubCenterTx();
        
        double rotationalRate = 0;
        
        if (hubVisible) {
            // === DIRECT VISION TRACKING ===
            // Hub is visible — use PD control on the corrected TX to aim at hub center
            
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
            
        } else {
            // === HUB NOT VISIBLE — HOLD CURRENT HEADING ===
            // Don't chase odometry estimates or spin wildly. Just hold still rotationally.
            // The driver can reposition to find the hub again, and aiming will resume
            // automatically once a hub tag comes back into view.
            rotationalRate = 0;
            lastTx = 0;  // Reset derivative so we don't get a spike when hub reappears
        }
        
        // Clamp rotation rate to keep this mode gentle
        rotationalRate = Math.max(-MAX_AIM_ROTATION_RATE, 
                        Math.min(MAX_AIM_ROTATION_RATE, rotationalRate));
        
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

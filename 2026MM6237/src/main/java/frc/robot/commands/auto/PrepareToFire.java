package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Command to prepare the robot to fire by aiming at the hub AprilTag.
 * 
 * Uses DIRECT Limelight TX feedback to rotate toward the target:
 * - When Limelight sees the hub tag, uses TX to calculate rotation needed
 * - Maintains last known target heading if tag is temporarily lost
 * - Simple proportional control for smooth, predictable aiming
 * 
 * This approach works regardless of odometry accuracy or field setup.
 * The robot will point directly at whatever AprilTag the Limelight detects.
 */
public class PrepareToFire extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    
    // Simple field-centric request for manual rotation control
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(0) // We handle our own deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // Rotation control - tuned for smooth aiming
    private static final double PROPORTIONAL_GAIN = 0.03;  // rad/s per degree of TX error (reduced from 0.05)
    private static final double DERIVATIVE_GAIN = 0.005;   // Damping to reduce oscillation
    private static final double MIN_ROTATION_SPEED = 0.15; // rad/s minimum to overcome friction (reduced)
    private static final double AIM_TOLERANCE_DEGREES = 3.0; // Stop rotating when within this (increased)
    private static final double FINE_AIM_THRESHOLD = 8.0;  // Below this, don't apply min speed (prevents oscillation)
    
    // State tracking
    private Rotation2d lastTargetHeading = null;
    private boolean hasEverSeenTarget = false;
    private double lastTx = 0;  // For derivative calculation

    public PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController) {
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
        SmartDashboard.putString("PrepareToFire/Status", "Searching for hub...");
        SmartDashboard.putBoolean("PrepareToFire/Aimed", false);
    }

    @Override
    public void execute() {
        // Get current robot heading
        Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
        
        // Check if Limelight sees the hub
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        double tx = limelight.getLastHubTx();  // Degrees offset from crosshair
        double distance = limelight.getLastHubDistance();
        int tagID = limelight.getLastHubTagID();
        
        double rotationalRate = 0;
        String aimState = "Unknown";
        
        if (hubVisible) {
            hasEverSeenTarget = true;
            aimState = "Tracking";
            
            // TX is the angle from crosshair to target
            // Positive TX = target is to the RIGHT of crosshair
            // We need to rotate RIGHT (negative in FRC convention) to center it
            
            // Calculate target heading: where we need to point to center the tag
            // Current heading + TX = heading that would center the tag
            lastTargetHeading = currentHeading.plus(Rotation2d.fromDegrees(tx));
            
            // Calculate derivative (rate of change of error) for damping
            double txDerivative = tx - lastTx;
            lastTx = tx;
            
            // Use TX directly for proportional control with derivative damping
            // If TX is positive (target right), we rotate right (negative rate in FRC)
            if (Math.abs(tx) > AIM_TOLERANCE_DEGREES) {
                // P term: proportional to error
                double pTerm = -tx * PROPORTIONAL_GAIN;
                
                // D term: resist rapid changes (negative because we want to slow down as we approach)
                double dTerm = -txDerivative * DERIVATIVE_GAIN;
                
                rotationalRate = pTerm + dTerm;
                
                // Only apply minimum speed if we're far from target (prevents oscillation when close)
                if (Math.abs(tx) > FINE_AIM_THRESHOLD) {
                    if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                        rotationalRate = MIN_ROTATION_SPEED;
                    } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                        rotationalRate = -MIN_ROTATION_SPEED;
                    }
                }
                aimState = "Rotating to target";
            } else {
                aimState = "AIMED";
                lastTx = 0; // Reset derivative when aimed
            }
            
            SmartDashboard.putString("PrepareToFire/Status", 
                Math.abs(tx) <= AIM_TOLERANCE_DEGREES ? "AIMED at Tag " + tagID : "Aiming at Tag " + tagID);
            SmartDashboard.putBoolean("PrepareToFire/Aimed", Math.abs(tx) <= AIM_TOLERANCE_DEGREES);
            
        } else if (hasEverSeenTarget && lastTargetHeading != null) {
            // Lost the target - rotate toward last known heading
            Rotation2d error = lastTargetHeading.minus(currentHeading);
            double errorDegrees = error.getDegrees();
            aimState = "Lost - Holding";
            
            if (Math.abs(errorDegrees) > AIM_TOLERANCE_DEGREES) {
                rotationalRate = errorDegrees * PROPORTIONAL_GAIN;
                
                if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                    rotationalRate = MIN_ROTATION_SPEED;
                } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                    rotationalRate = -MIN_ROTATION_SPEED;
                }
                aimState = "Lost - Rotating to last known";
            }
            
            SmartDashboard.putString("PrepareToFire/Status", "Target lost - holding heading");
            SmartDashboard.putBoolean("PrepareToFire/Aimed", false);
        } else {
            // Never seen target
            aimState = "Searching";
            SmartDashboard.putString("PrepareToFire/Status", "Searching for hub...");
            SmartDashboard.putBoolean("PrepareToFire/Aimed", false);
        }
        
        // Clamp rotation rate
        double rawRotationalRate = rotationalRate;
        rotationalRate = Math.max(-Constants.TempSwerve.MaxAngularRate, 
                        Math.min(Constants.TempSwerve.MaxAngularRate, rotationalRate));
        
        // === COMPREHENSIVE DEBUG OUTPUTS ===
        // Limelight data
        SmartDashboard.putBoolean("PrepareToFire/Hub Visible", hubVisible);
        SmartDashboard.putNumber("PrepareToFire/TX (deg)", tx);
        SmartDashboard.putNumber("PrepareToFire/Hub Tag ID", tagID);
        SmartDashboard.putNumber("PrepareToFire/Distance To Hub (m)", distance > 0 ? distance : -1);
        
        // DIRECTION DEBUG - Critical for diagnosing wrong-way rotation
        String txDirection = tx > 0 ? "RIGHT" : (tx < 0 ? "LEFT" : "CENTER");
        String rotDirection = rotationalRate > 0 ? "CCW (left)" : (rotationalRate < 0 ? "CW (right)" : "STOPPED");
        SmartDashboard.putString("PrepareToFire/TX Direction", txDirection);
        SmartDashboard.putString("PrepareToFire/Rotation Direction", rotDirection);
        SmartDashboard.putString("PrepareToFire/Direction Logic", 
            "TX=" + txDirection + " -> Rotating " + rotDirection);
        
        // Robot state
        SmartDashboard.putNumber("PrepareToFire/Current Heading (deg)", currentHeading.getDegrees());
        SmartDashboard.putNumber("PrepareToFire/Target Heading (deg)", 
            lastTargetHeading != null ? lastTargetHeading.getDegrees() : 0);
        
        // Control outputs
        SmartDashboard.putNumber("PrepareToFire/Rotation Rate (rad/s)", rotationalRate);
        SmartDashboard.putNumber("PrepareToFire/Raw Rotation Rate (rad/s)", rawRotationalRate);
        SmartDashboard.putNumber("PrepareToFire/Aim Error (deg)", hubVisible ? tx : 
            (lastTargetHeading != null ? lastTargetHeading.minus(currentHeading).getDegrees() : 0));
        
        // State tracking
        SmartDashboard.putBoolean("PrepareToFire/Has Ever Seen Target", hasEverSeenTarget);
        SmartDashboard.putString("PrepareToFire/Aim State", aimState);
        
        // Control parameters (for tuning reference)
        SmartDashboard.putNumber("PrepareToFire/PARAM P Gain", PROPORTIONAL_GAIN);
        SmartDashboard.putNumber("PrepareToFire/PARAM Min Speed (rad/s)", MIN_ROTATION_SPEED);
        SmartDashboard.putNumber("PrepareToFire/PARAM Tolerance (deg)", AIM_TOLERANCE_DEGREES);
        
        // Apply rotation only - no translation
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        SmartDashboard.putString("PrepareToFire/Status", "Command ended");
        SmartDashboard.putBoolean("PrepareToFire/Aimed", false);
    }
}

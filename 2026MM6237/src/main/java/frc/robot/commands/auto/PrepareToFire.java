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
 * Autonomous command to prepare the robot to fire.
 * 
 * Rotates the robot to face the hub AprilTag.
 * Uses background-tracked distance from LimelightSubsystem (no button press needed).
 * Maintains last known target heading even if tag is temporarily lost.
 * Does NOT spin up shooter - only aims the robot for testing.
 */
public class PrepareToFire extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final SwerveRequest.FieldCentric driveRequest;
    
    // Rotation control constants - tuned for smooth, stable aiming
    private static final double PROPORTIONAL_GAIN = 2.5;  // rad/s per radian of error
    private static final double MIN_ROTATION_SPEED = 0.3; // rad/s minimum when error > threshold
    private static final double ERROR_THRESHOLD = 0.05;   // ~2.9 degrees - deadband near target
    
    // State tracking
    private Rotation2d targetRotation;
    private boolean hasEverSeenTarget = false;

    public PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        
        // Create a field-centric request for manual rotation control (like SnapToNearestAngleCommand)
        this.driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
            .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        // DO NOT require shooter - we're just reading distance
        // Require drivetrain so we can rotate
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Get current robot rotation to start from
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        targetRotation = currentRotation;  // Start with current heading
        hasEverSeenTarget = false;
        
        SmartDashboard.putString("PrepareToFire/Status", "Aiming started");
    }

    @Override
    public void execute() {
        // Check if hub is currently visible (background tracking handles detection)
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        
        // Get current robot rotation
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        
        // Always show current absolute robot angle for debugging
        SmartDashboard.putNumber("PrepareToFire/DEBUG Robot Absolute Angle (deg)", currentRotation.getDegrees());
        
        // Update target heading if hub is visible
        if (hubVisible) {
            // Get the tracked hub data (updated in background by periodic())
            double tx = limelight.getLastHubTx();
            int tagID = limelight.getLastHubTagID();
            
            // Calculate target rotation: current rotation + TX offset
            // TX is the horizontal angle offset from crosshair to target
            // This gives us the absolute field angle needed to face the hub
            targetRotation = currentRotation.plus(Rotation2d.fromDegrees(tx));
            hasEverSeenTarget = true;
            
            SmartDashboard.putNumber("PrepareToFire/Target Heading (deg)", targetRotation.getDegrees());
            SmartDashboard.putNumber("PrepareToFire/Current Heading (deg)", currentRotation.getDegrees());
            SmartDashboard.putNumber("PrepareToFire/TX Offset (deg)", tx);
            SmartDashboard.putNumber("PrepareToFire/DEBUG Target Absolute Angle (deg)", targetRotation.getDegrees());
            SmartDashboard.putString("PrepareToFire/Status", "Aiming at Tag " + tagID);
        } else {
            // No hub visible - maintain last known heading
            if (hasEverSeenTarget) {
                SmartDashboard.putString("PrepareToFire/Status", "Maintaining last heading (tag lost)");
                SmartDashboard.putNumber("PrepareToFire/DEBUG Target Absolute Angle (deg)", targetRotation.getDegrees());
            } else {
                SmartDashboard.putString("PrepareToFire/Status", "Waiting for hub detection...");
            }
        }
        
        // Calculate the shortest rotation error (like SnapToNearestAngleCommand does)
        Rotation2d rotationError = targetRotation.minus(currentRotation);
        double errorRadians = rotationError.getRadians();
        
        // Debug: show what we're working with
        SmartDashboard.putBoolean("PrepareToFire/DEBUG Has Target", hasEverSeenTarget);
        SmartDashboard.putNumber("PrepareToFire/DEBUG Raw Error (rad)", errorRadians);
        SmartDashboard.putNumber("PrepareToFire/DEBUG Raw Error (deg)", Math.toDegrees(errorRadians));
        
        // Calculate desired rotational rate with proportional control
        double rotationalRate = errorRadians * PROPORTIONAL_GAIN;
        
        SmartDashboard.putNumber("PrepareToFire/DEBUG Proportional Rate (before limits)", rotationalRate);
        
        // Add a minimum rotation speed to overcome friction when there's significant error
        if (Math.abs(errorRadians) > ERROR_THRESHOLD) {
            // Apply minimum speed in the direction needed
            if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                rotationalRate = MIN_ROTATION_SPEED;
            } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                rotationalRate = -MIN_ROTATION_SPEED;
            }
        } else {
            // Near the target, stop rotating
            rotationalRate = 0;
        }
        
        SmartDashboard.putNumber("PrepareToFire/DEBUG Rate After Min Speed", rotationalRate);
        
        // Clamp to maximum angular rate
        rotationalRate = Math.max(
            -Constants.TempSwerve.MaxAngularRate,
            Math.min(Constants.TempSwerve.MaxAngularRate, rotationalRate)
        );
        
        // Display error as the absolute difference (easier to understand)
        double errorDegrees = Math.toDegrees(errorRadians);
        SmartDashboard.putNumber("PrepareToFire/Rotation Error (deg)", errorDegrees);
        SmartDashboard.putNumber("PrepareToFire/Rotation Rate (rad/s)", rotationalRate);
        SmartDashboard.putNumber("PrepareToFire/DEBUG Rotation Error (rad)", errorRadians);
        SmartDashboard.putNumber("PrepareToFire/DEBUG Final Rotation Rate", rotationalRate);
        
        // Apply ONLY rotation control - no translation at all
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public boolean isFinished() {
        // Never finish automatically - operator holds button to read distance
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop rotating and translating - return control to driver
        SmartDashboard.putString("PrepareToFire/Status", "Command ended");
    }

    /**
     * Calculates the required shooter RPM based on distance from the hub.
     * This is a placeholder implementation that can be tuned based on testing.
     * 
     * @param distanceMeters Distance to the hub in meters
     * @return Required shooter RPM
     */
    private double calculateShooterRPM(double distanceMeters) {
        // Placeholder linear relationship: adjust these values based on tuning
        // Example: closer = lower RPM, farther = higher RPM
        double minDistance = Constants.Shooter.kAutoMinShootingDistanceMeters;
        double maxDistance = Constants.Shooter.kAutoMaxShootingDistanceMeters;
        double minRPM = Constants.Shooter.kAutoMinShooterRPM;
        double maxRPM = Constants.Shooter.kAutoMaxShooterRPM;
        
        if (distanceMeters <= minDistance) {
            return minRPM;
        } else if (distanceMeters >= maxDistance) {
            return maxRPM;
        } else {
            // Linear interpolation between min and max RPM
            double fraction = (distanceMeters - minDistance) / (maxDistance - minDistance);
            return minRPM + (maxRPM - minRPM) * fraction;
        }
    }
}

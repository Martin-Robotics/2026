package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

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
 * Rotates the robot to face the hub AprilTag while allowing driver to strafe.
 * Uses background-tracked distance from LimelightSubsystem (no button press needed).
 * Maintains last known target heading even if tag is temporarily lost.
 * Does NOT spin up shooter - only aims the robot for testing.
 */
public class PrepareToFire extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final SwerveRequest.FieldCentricFacingAngle aimRequest;
    
    // Rotation control constants - tuned for smooth, stable aiming
    private static final double ROTATION_kP = 3.0;  // Proportional gain for rotation (reduced from 5.0 to eliminate jerking)
    private static final double ROTATION_kD = 0.1;  // Derivative gain for rotation (reduced from 0.3 to reduce oscillation)
    
    // State tracking
    private double lastTargetHeadingRadians;
    private boolean hasEverSeenTarget = false;

    public PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        
        // Create a field-centric request that maintains heading toward target
        this.aimRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
        
        // Configure the heading controller
        aimRequest.HeadingController.setPID(ROTATION_kP, 0, ROTATION_kD);
        aimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        
        // DO NOT require shooter - we're just reading distance
        // Require drivetrain so we can rotate
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset state tracking
        hasEverSeenTarget = false;
        lastTargetHeadingRadians = drivetrain.getRotation3d().toRotation2d().getRadians();
        
        SmartDashboard.putString("PrepareToFire/Status", "Aiming started");
    }

    @Override
    public void execute() {
        // NO driver strafe inputs - PrepareToFire only rotates, doesn't translate
        // Driver maintains normal control of translation via default command
        
        // Check if hub is currently visible (background tracking handles detection)
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        
        // Update target heading if hub is visible
        if (hubVisible) {
            // Get the tracked hub data (updated in background by periodic())
            double tx = limelight.getLastHubTx();
            int tagID = limelight.getLastHubTagID();
            double currentHeadingRadians = drivetrain.getRotation3d().toRotation2d().getRadians();
            
            // Only update target if tx has changed significantly (reduce jitter)
            // Calculate what the absolute field-relative target should be
            double txRadians = Math.toRadians(tx);
            double newTargetHeadingRadians = currentHeadingRadians + txRadians;
            
            // Update target heading (this will smoothly converge as robot rotates)
            lastTargetHeadingRadians = newTargetHeadingRadians;
            hasEverSeenTarget = true;
            
            SmartDashboard.putNumber("PrepareToFire/Target Heading (deg)", Math.toDegrees(lastTargetHeadingRadians));
            SmartDashboard.putNumber("PrepareToFire/Current Heading (deg)", Math.toDegrees(currentHeadingRadians));
            SmartDashboard.putNumber("PrepareToFire/Heading Error (deg)", tx);
            SmartDashboard.putString("PrepareToFire/Status", "Aiming at Tag " + tagID);
        } else {
            // No hub visible - maintain last known heading or show warning
            if (hasEverSeenTarget) {
                SmartDashboard.putString("PrepareToFire/Status", "Maintaining last heading (tag lost)");
            } else {
                SmartDashboard.putString("PrepareToFire/Status", "Waiting for hub detection...");
            }
        }
        
        // Apply ONLY rotation control - no translation at all
        drivetrain.setControl(aimRequest
            .withVelocityX(0)  // No translation
            .withVelocityY(0)  // No translation
            .withTargetDirection(new edu.wpi.first.math.geometry.Rotation2d(lastTargetHeadingRadians)));
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

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
 * Reads distance from the hub using the Limelight and displays it on SmartDashboard.
 * Rotates the robot to face the hub AprilTag while allowing driver to strafe.
 * Maintains last known target heading even if tag is temporarily lost.
 * Does NOT spin up shooter - only reads distance and aims for testing.
 */
public class PrepareToFire extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final SwerveRequest.FieldCentricFacingAngle aimRequest;
    
    // Rotation control constants
    private static final double ROTATION_kP = 8.0;  // Proportional gain for rotation
    private static final double ROTATION_kD = 0.5;  // Derivative gain for rotation
    
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
        
        // Just read distance - don't calculate RPM or spin up shooter
        double distanceToHub = -1.0; // Default to -1 if no target detected
        int detectedTagID = -1;
        
        if (limelight.hasValidTarget()) {
            // Try red alliance hub first (ID 10)
            distanceToHub = limelight.getDistanceToTag(Constants.Auto.kRedHubAprilTagID);
            if (distanceToHub > 0) {
                detectedTagID = Constants.Auto.kRedHubAprilTagID;
            } else {
                // Try blue alliance hub (ID 26)
                distanceToHub = limelight.getDistanceToTag(Constants.Auto.kBlueHubAprilTagID);
                if (distanceToHub > 0) {
                    detectedTagID = Constants.Auto.kBlueHubAprilTagID;
                }
            }
            
            if (distanceToHub > 0) {
                // Output distance to SmartDashboard
                SmartDashboard.putNumber("PrepareToFire/Distance To Hub (m)", distanceToHub);
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", true);
                SmartDashboard.putNumber("PrepareToFire/Hub Tag ID", detectedTagID);
                SmartDashboard.putString("PrepareToFire/Status", "Target locked - Tag " + detectedTagID);
                
                // Calculate what RPM would be (but don't use it)
                double calculatedRPM = calculateShooterRPM(distanceToHub);
                SmartDashboard.putNumber("PrepareToFire/Calculated RPM", calculatedRPM);
            } else {
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
                SmartDashboard.putString("PrepareToFire/Status", "Hub tags not visible (looking for 10 or 26)");
            }
        } else {
            SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
            SmartDashboard.putString("PrepareToFire/Status", "No Limelight target");
        }
        
        // DO NOT spin up shooter - just reading distance
    }

    @Override
    public void execute() {
        // Get driver strafe inputs (scaled with speed multipliers like DriverController)
        double speedMultiplier = 1.0; // Default 35% speed
        boolean slowSpeed = driverController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold;
        boolean fastSpeed = driverController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold;
        
        if (slowSpeed) {
            speedMultiplier = 15.0 / 35.0; // Scale to 15%
        } else if (fastSpeed) {
            speedMultiplier = 55.0 / 35.0; // Scale to 55%
        }
        
        // Read driver strafe inputs (inverted to match DriverController)
        double strafeX = -1.0 * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier;
        double strafeY = -1.0 * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier;
        
        // Debug: Show if Limelight has any target at all
        boolean hasTarget = limelight.hasValidTarget();
        SmartDashboard.putBoolean("PrepareToFire/DEBUG HasValidTarget", hasTarget);
        SmartDashboard.putNumber("PrepareToFire/DEBUG Fiducial Count", limelight.getDetectedFiducialCount());
        
        // Get the best hub tag (prioritizes center, closest to crosshair)
        int visibleTagID = limelight.getBestHubTagID();
        SmartDashboard.putNumber("PrepareToFire/DEBUG Visible Tag ID", visibleTagID);
        
        // Check if we see a hub tag and update target heading
        if (hasTarget && (visibleTagID == Constants.Auto.kRedHubAprilTagID || visibleTagID == Constants.Auto.kBlueHubAprilTagID)) {
            // Get the horizontal angle to the tag
            double tx = limelight.getTxToTag(visibleTagID);
            SmartDashboard.putNumber("PrepareToFire/DEBUG TX to Tag", tx);
            
            // Calculate target heading (current heading + tx offset)
            double currentHeadingRadians = drivetrain.getRotation3d().toRotation2d().getRadians();
            double txRadians = Math.toRadians(tx);
            lastTargetHeadingRadians = currentHeadingRadians + txRadians;
            hasEverSeenTarget = true;
            
            SmartDashboard.putNumber("PrepareToFire/Target Heading (deg)", Math.toDegrees(lastTargetHeadingRadians));
            SmartDashboard.putNumber("PrepareToFire/Current Heading (deg)", Math.toDegrees(currentHeadingRadians));
            SmartDashboard.putNumber("PrepareToFire/Heading Error (deg)", tx);
            
            // Use simple distance calculation as fallback
            double simpleDistance = limelight.getSimpleDistance();
            SmartDashboard.putNumber("PrepareToFire/DEBUG Simple Distance", simpleDistance);
            
            // Try the complex method too
            double complexDistance = limelight.getDistanceToTag(visibleTagID);
            SmartDashboard.putNumber("PrepareToFire/DEBUG Complex Distance", complexDistance);
            
            // Use whichever method works
            double distanceToHub = complexDistance > 0 ? complexDistance : simpleDistance;
            
            if (distanceToHub > 0) {
                SmartDashboard.putNumber("PrepareToFire/Distance To Hub (m)", distanceToHub);
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", true);
                SmartDashboard.putNumber("PrepareToFire/Hub Tag ID", visibleTagID);
                SmartDashboard.putString("PrepareToFire/Status", "Aiming at Tag " + visibleTagID);
                
                // Calculate what RPM would be (but don't use it)
                double calculatedRPM = calculateShooterRPM(distanceToHub);
                SmartDashboard.putNumber("PrepareToFire/Calculated RPM", calculatedRPM);
            } else {
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
                SmartDashboard.putString("PrepareToFire/Status", "Cannot calculate distance");
            }
        } else {
            // No hub tag visible - maintain last known heading or show warning
            if (hasEverSeenTarget) {
                SmartDashboard.putString("PrepareToFire/Status", "Maintaining last heading (tag lost)");
            } else {
                SmartDashboard.putString("PrepareToFire/Status", "Searching for hub tags (10 or 26)");
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
            }
        }
        
        // Always apply rotation control with driver strafe, using last known target heading
        drivetrain.setControl(aimRequest
            .withVelocityX(strafeX)  // Driver forward/back
            .withVelocityY(strafeY)  // Driver left/right
            .withTargetDirection(new edu.wpi.first.math.geometry.Rotation2d(lastTargetHeadingRadians)));
        
        // DO NOT update shooter status - we're not running the shooter
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

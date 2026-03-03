package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to prepare the robot to fire.
 * 
 * Reads distance from the hub using the Limelight and displays it on SmartDashboard.
 * Does NOT spin up shooter - only reads and displays distance for testing.
 */
public class PrepareToFire extends Command {
    private final LimelightSubsystem6237 limelight;

    public PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight) {
        this.limelight = limelight;
        // DO NOT require shooter - we're just reading distance
    }

    @Override
    public void initialize() {
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
        // Continuously update distance to SmartDashboard while command is running
        
        // Debug: Show if Limelight has any target at all
        boolean hasTarget = limelight.hasValidTarget();
        SmartDashboard.putBoolean("PrepareToFire/DEBUG HasValidTarget", hasTarget);
        SmartDashboard.putNumber("PrepareToFire/DEBUG Fiducial Count", limelight.getDetectedFiducialCount());
        
        if (!hasTarget) {
            SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
            SmartDashboard.putString("PrepareToFire/Status", "No Limelight target");
            return;
        }
        
        // Get the visible tag ID using simple method
        int visibleTagID = limelight.getVisibleTagID();
        SmartDashboard.putNumber("PrepareToFire/DEBUG Visible Tag ID", visibleTagID);
        
        // Check if it's one of our hub tags (10 or 26)
        if (visibleTagID == Constants.Auto.kRedHubAprilTagID || visibleTagID == Constants.Auto.kBlueHubAprilTagID) {
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
                SmartDashboard.putString("PrepareToFire/Status", "Target locked - Tag " + visibleTagID);
                
                // Calculate what RPM would be (but don't use it)
                double calculatedRPM = calculateShooterRPM(distanceToHub);
                SmartDashboard.putNumber("PrepareToFire/Calculated RPM", calculatedRPM);
            } else {
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
                SmartDashboard.putString("PrepareToFire/Status", "Cannot calculate distance");
            }
        } else {
            SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
            SmartDashboard.putString("PrepareToFire/Status", "Visible tag " + visibleTagID + " is not a hub (need 10 or 26)");
        }
        
        // DO NOT update shooter status - we're not running the shooter
    }

    @Override
    public boolean isFinished() {
        // Never finish automatically - operator holds button to read distance
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to stop - we didn't spin up any motors
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

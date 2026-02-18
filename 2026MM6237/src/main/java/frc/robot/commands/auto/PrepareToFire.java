package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to prepare the robot to fire.
 * 
 * Spins up the shooter rollers at an RPM determined by distance from the hub,
 * rotates the robot to face the hub, and verifies the shooter is at speed.
 * The rollers continue running when the command ends.
 */
public class PrepareToFire extends Command {
    private final Shooter shooter;
    private final LimelightSubsystem6237 limelight;

    public PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Determine shooter RPM based on distance
        double shooterRPM = Constants.Shooter.kAutoDefaultShooterRPM;
        
        if (limelight.hasValidTarget()) {
            double distanceToHub = limelight.getDistanceToTag(Constants.Auto.kHubAprilTagID);
            if (distanceToHub > 0) {
                // Calculate shooter RPM based on distance
                // This is a placeholder formula - tune based on your robot's ballistics
                shooterRPM = calculateShooterRPM(distanceToHub);
            }
        }
        
        // Spin up shooter rollers
        shooter.setRPM(shooterRPM);
    }

    @Override
    public void execute() {
        // Rollers continue spinning and ramping up to speed
    }

    @Override
    public boolean isFinished() {
        // Command finishes when shooter reaches target velocity
        return shooter.isVelocityWithinTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        // Rollers continue spinning when command ends
        // They will be stopped by the Fire command
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

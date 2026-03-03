package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to fire the note at the hub.
 * 
 * Waits for the shooter to reach speed and hood to reach position,
 * then runs the feeder motor for a set amount of time to feed the note through the shooter.
 * The feeder speed is determined by distance from the hub.
 * After firing, stops both the feeder and shooter rollers.
 */
public class Fire extends Command {
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final LimelightSubsystem6237 limelight;
    
    private final Timer fireTimer = new Timer();
    private boolean readyToFire = false;

    public Fire(Feeder feeder, Shooter shooter, Hood hood, LimelightSubsystem6237 limelight) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.limelight = limelight;
        addRequirements(feeder, shooter);
    }

    @Override
    public void initialize() {
        fireTimer.reset();
        readyToFire = false;
    }

    @Override
    public void execute() {
        // Wait for both shooter to be at speed AND hood to be at position
        if (!readyToFire) {
            if (shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance()) {
                // Both shooter and hood are ready, start firing
                readyToFire = true;
                fireTimer.start();
                
                // Determine feeder speed based on distance
                double feederPercentOutput = Constants.Feeder.kAutoDefaultFeederPercentOutput;
                
                if (limelight.hasValidTarget()) {
                    double distanceToHub = limelight.getDistanceToTag(Constants.Auto.kHubAprilTagID);
                    if (distanceToHub > 0) {
                        // Calculate feeder speed based on distance
                        feederPercentOutput = calculateFeederSpeed(distanceToHub);
                    }
                }
                
                // Start feeder at calculated speed
                feeder.setPercentOutput(feederPercentOutput);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes after firing duration has elapsed (only after we started firing)
        return readyToFire && fireTimer.hasElapsed(Constants.Feeder.kAutoFireDurationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        fireTimer.stop();
        // Stop feeder
        feeder.setPercentOutput(0.0);
        // Stop shooter rollers
        shooter.stop();
    }

    /**
     * Calculates the required feeder speed based on distance from the hub.
     * This is a placeholder implementation that can be tuned based on testing.
     * 
     * @param distanceMeters Distance to the hub in meters
     * @return Feeder percent output (0.0 to 1.0)
     */
    private double calculateFeederSpeed(double distanceMeters) {
        // Placeholder linear relationship: adjust these values based on tuning
        double minDistance = Constants.Feeder.kAutoMinFeederDistanceMeters;
        double maxDistance = Constants.Feeder.kAutoMaxFeederDistanceMeters;
        double minPercentOutput = Constants.Feeder.kAutoMinFeederPercentOutput;
        double maxPercentOutput = Constants.Feeder.kAutoMaxFeederPercentOutput;
        
        if (distanceMeters <= minDistance) {
            return minPercentOutput;
        } else if (distanceMeters >= maxDistance) {
            return maxPercentOutput;
        } else {
            // Linear interpolation between min and max speeds
            double fraction = (distanceMeters - minDistance) / (maxDistance - minDistance);
            return minPercentOutput + (maxPercentOutput - minPercentOutput) * fraction;
        }
    }
}

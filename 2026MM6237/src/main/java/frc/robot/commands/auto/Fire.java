package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to fire the note at the hub.
 * 
 * Runs the feeder motor for a set amount of time to feed the note through the shooter.
 * The feeder speed is determined by distance from the hub.
 * After firing, stops both the feeder and shooter rollers.
 */
public class Fire extends Command {
    private final Feeder feeder;
    private final Shooter shooter;
    private final LimelightSubsystem6237 limelight;
    
    private final Timer fireTimer = new Timer();

    public Fire(Feeder feeder, Shooter shooter, LimelightSubsystem6237 limelight) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(feeder, shooter);
    }

    @Override
    public void initialize() {
        fireTimer.reset();
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

    @Override
    public void execute() {
        // Feeder is running, shooter is already spinning from PrepareToFire
    }

    @Override
    public boolean isFinished() {
        // Command finishes after firing duration has elapsed
        return fireTimer.hasElapsed(Constants.Feeder.kAutoFireDurationSeconds);
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

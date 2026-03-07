package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to fire the note at the hub.
 * 
 * Similar to Fire command but designed for autonomous use:
 * - Waits for shooter to reach speed and hood to reach position
 * - Runs the feeder for a fixed duration (kAutoFireRunTimeSeconds) after shooter is ready
 * - Automatically ends after the firing duration completes
 * 
 * The feeder speed is determined by distance from the hub (from Limelight or default).
 * After firing, stops both the feeder and shooter rollers.
 */
public class FireAutonomous extends Command {
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final LimelightSubsystem6237 limelight;
    
    private final Timer fireTimer = new Timer();
    private boolean readyToFire = false;

    public FireAutonomous(Feeder feeder, Shooter shooter, Hood hood, LimelightSubsystem6237 limelight) {
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
        SmartDashboard.putString("FireAutonomous/Status", "Waiting for shooter...");
    }

    @Override
    public void execute() {
        // Wait for both shooter to be at speed AND hood to be at position
        if (!readyToFire) {
            boolean shooterReady = shooter.isVelocityWithinTolerance();
            boolean hoodReady = hood.isPositionWithinTolerance();
            
            SmartDashboard.putBoolean("FireAutonomous/Shooter Ready", shooterReady);
            SmartDashboard.putBoolean("FireAutonomous/Hood Ready", hoodReady);
            
            if (shooterReady && hoodReady) {
                // Both shooter and hood are ready, start firing
                readyToFire = true;
                fireTimer.start();
                
                // Determine feeder speed based on distance
                double feederPercentOutput = Constants.Feeder.kAutoDefaultFeederPercentOutput;
                
                // Use cached distance from LimelightSubsystem (already tracked in background)
                double distanceToHub = limelight.getLastHubDistance();
                if (distanceToHub > 0) {
                    feederPercentOutput = calculateFeederSpeed(distanceToHub);
                    SmartDashboard.putNumber("FireAutonomous/Distance Used (m)", distanceToHub);
                }
                
                // Start feeder at calculated speed
                feeder.setPercentOutput(feederPercentOutput);
                SmartDashboard.putNumber("FireAutonomous/Feeder Output", feederPercentOutput);
                SmartDashboard.putString("FireAutonomous/Status", "FIRING!");
            }
        } else {
            // Show countdown
            double remaining = Constants.Auto.kAutoFireRunTimeSeconds - fireTimer.get();
            SmartDashboard.putNumber("FireAutonomous/Time Remaining", remaining);
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes after firing duration has elapsed (only after we started firing)
        return readyToFire && fireTimer.hasElapsed(Constants.Auto.kAutoFireRunTimeSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        fireTimer.stop();
        // Stop feeder
        feeder.setPercentOutput(0.0);
        // Stop shooter rollers
        shooter.stop();
        
        if (interrupted) {
            SmartDashboard.putString("FireAutonomous/Status", "Interrupted");
        } else {
            SmartDashboard.putString("FireAutonomous/Status", "Complete");
        }
    }

    /**
     * Calculates the required feeder speed based on distance from the hub.
     * 
     * @param distanceMeters Distance to the hub in meters
     * @return Feeder percent output (0.0 to 1.0)
     */
    private double calculateFeederSpeed(double distanceMeters) {
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

package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.WCP.PrepareShotCommand.Shot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to fire at the hub.
 * 
 * Uses the same interpolation table as PrepareStaticShotCommand (teleop RT fire):
 * - Reads distance from Limelight background tracking
 * - Looks up RPM and hood position from interpolation table
 * - Spins up shooter and positions hood
 * - Once both are ready, engages feeder and floor for a fixed duration
 * - Automatically ends after firing duration completes
 * 
 * Interpolation table tuned 2026-03-07 (right shooter only - awaiting parts for left/middle).
 */
public class FireAutonomous extends Command {
    // Same interpolation table as PrepareStaticShotCommand
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        // Interpolation table - tuned 2026-03-07 using ShooterTuningCommand
        // Distances are ACTUAL Limelight-reported values (not physical tape measure)
        // CALIBRATION NOTE: Right shooter only - left/middle awaiting parts
        distanceToShotMap.put(Inches.of(70.9),  new Shot(2700, 0.25));  // ~1.5m physical, LL reads 1.8m
        distanceToShotMap.put(Inches.of(114.2), new Shot(3000, 0.40));  // ~2.5m physical, LL reads 2.9m
        distanceToShotMap.put(Inches.of(149.6), new Shot(3400, 0.50));  // ~3.5m physical, LL reads 3.8m
        distanceToShotMap.put(Inches.of(185.0), new Shot(3800, 0.50));  // ~4.5m physical, LL reads 4.7m
        distanceToShotMap.put(Inches.of(212.6), new Shot(4100, 0.62));  // ~5.0m physical, LL reads 5.4m
    }

    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Floor floor;
    private final LimelightSubsystem6237 limelight;
    
    private final Timer fireTimer = new Timer();
    private boolean shooterAtSpeed = false;

    public FireAutonomous(Feeder feeder, Shooter shooter, Hood hood, Floor floor, LimelightSubsystem6237 limelight) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.floor = floor;
        this.limelight = limelight;
        addRequirements(feeder, shooter, hood, floor);
    }

    @Override
    public void initialize() {
        fireTimer.reset();
        shooterAtSpeed = false;
        SmartDashboard.putString("FireAutonomous/Status", "Spinning up...");
    }

    @Override
    public void execute() {
        // Get distance from Limelight background tracking
        double detectedDistance = limelight.getLastHubDistance();
        Distance actualDistance = detectedDistance > 0 
            ? Meters.of(detectedDistance) 
            : Meters.of(3.0); // Fallback to 3m if no hub seen
        
        // Look up RPM and hood from interpolation table
        final Shot shot = distanceToShotMap.get(actualDistance);
        
        // Always spin up shooter and position hood
        shooter.setRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        
        SmartDashboard.putNumber("FireAutonomous/Distance (m)", actualDistance.in(Meters));
        SmartDashboard.putNumber("FireAutonomous/Target RPM", shot.shooterRPM);
        SmartDashboard.putNumber("FireAutonomous/Target Hood", shot.hoodPosition);
        
        if (!shooterAtSpeed) {
            boolean shooterReady = shooter.isVelocityWithinTolerance();
            boolean hoodReady = hood.isPositionWithinTolerance();
            
            SmartDashboard.putBoolean("FireAutonomous/Shooter Ready", shooterReady);
            SmartDashboard.putBoolean("FireAutonomous/Hood Ready", hoodReady);
            
            if (shooterReady && hoodReady) {
                // Both ready — start firing
                shooterAtSpeed = true;
                fireTimer.start();
                feeder.set(Feeder.Speed.FEED);
                floor.set(Floor.Speed.FEED);
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
        return shooterAtSpeed && fireTimer.hasElapsed(Constants.Auto.kAutoFireRunTimeSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        fireTimer.stop();
        shooter.stop();
        feeder.setPercentOutput(0);
        floor.set(Floor.Speed.STOP);
        
        if (interrupted) {
            SmartDashboard.putString("FireAutonomous/Status", "Interrupted");
        } else {
            SmartDashboard.putString("FireAutonomous/Status", "Complete");
        }
    }
}

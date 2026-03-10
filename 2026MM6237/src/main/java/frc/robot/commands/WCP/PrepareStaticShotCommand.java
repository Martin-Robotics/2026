package frc.robot.commands.WCP;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.WCP.PrepareShotCommand.Shot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/**
 * Prepares the shooter and feeds game pieces at a known static distance.
 * Unlike PrepareShotCommand, this does not use field position or odometry.
 * Spins up the shooter, positions the hood, and engages the feeder and floor once at speed.
 */
public class PrepareStaticShotCommand extends Command {
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
        //
        // CALIBRATION NOTE: These values were tuned with RIGHT SHOOTER ONLY
        // (correct roller wheel installed). Left and middle are awaiting parts.
        // RPM values will likely need adjustment once all three wheels are installed.
        distanceToShotMap.put(Inches.of(70.9),  new Shot(2700, 0.25));  // ~1.5m physical, LL reads 1.8m
        distanceToShotMap.put(Inches.of(114.2), new Shot(3000, 0.40));  // ~2.5m physical, LL reads 2.9m
        distanceToShotMap.put(Inches.of(149.6), new Shot(3400, 0.50));  // ~3.5m physical, LL reads 3.8m
        distanceToShotMap.put(Inches.of(185.0), new Shot(3800, 0.50));  // ~4.5m physical, LL reads 4.7m
        distanceToShotMap.put(Inches.of(212.6), new Shot(4100, 0.62));  // ~5.0m physical, LL reads 5.4m
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final Floor floor;
    private final Distance targetDistance;
    private final boolean useDetectedDistance;
    private final frc.robot.subsystems.LimelightSubsystem6237 limelight;
    private boolean shooterAtSpeed = false;

    /**
     * Creates a command to prepare for a shot at a static distance.
     * 
     * @param shooter The shooter subsystem
     * @param hood The hood subsystem
     * @param feeder The feeder subsystem
     * @param floor The floor subsystem
     * @param distanceMeters The distance to the target in meters
     */
    public PrepareStaticShotCommand(Shooter shooter, Hood hood, Feeder feeder, Floor floor, double distanceMeters) {
        this(shooter, hood, feeder, floor, distanceMeters, false, null);
    }
    
    /**
     * Creates a command to prepare for a shot, optionally using detected distance from Limelight.
     * 
     * @param shooter The shooter subsystem
     * @param hood The hood subsystem
     * @param feeder The feeder subsystem
     * @param floor The floor subsystem
     * @param fallbackDistanceMeters The fallback distance if no detected distance available
     * @param useDetectedDistance If true, uses last detected distance from Limelight background tracking
     * @param limelight The Limelight subsystem (required if useDetectedDistance is true)
     */
    public PrepareStaticShotCommand(Shooter shooter, Hood hood, Feeder feeder, Floor floor, double fallbackDistanceMeters, boolean useDetectedDistance, frc.robot.subsystems.LimelightSubsystem6237 limelight) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.floor = floor;
        this.targetDistance = Meters.of(fallbackDistanceMeters);
        this.useDetectedDistance = useDetectedDistance;
        this.limelight = limelight;
        addRequirements(shooter, hood, feeder, floor);
    }

    @Override
    public void initialize() {
        shooterAtSpeed = false;
    }

    public boolean isReadyToShoot() {
        return shooterAtSpeed;
    }

    @Override
    public void execute() {
        // Determine which distance to use
        Distance actualDistance = targetDistance;
        
        if (useDetectedDistance && limelight != null) {
            // Use background-tracked distance from Limelight
            double detectedDistance = limelight.getLastHubDistance();
            
            if (detectedDistance > 0) {
                actualDistance = Meters.of(detectedDistance);
            }
        }
        
        final Shot shot = distanceToShotMap.get(actualDistance);
        
        // Always spin up shooter and position hood
        shooter.setRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        
        // Check if both shooter has reached speed AND hood has reached position
        if (!shooterAtSpeed && shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance()) {
            shooterAtSpeed = true;
            // Engage feeder and floor once both shooter and hood are ready
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
        }
        
        // Debug outputs (commented out after testing)
        // SmartDashboard.putNumber("Static Distance to Target (inches)", targetDistance.in(Inches));
        // SmartDashboard.putNumber("Target Shooter RPM", shot.shooterRPM);
        // SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
        // SmartDashboard.putBoolean("Shooter At Speed", shooterAtSpeed);
        // SmartDashboard.putBoolean("Hood At Position", hood.isPositionWithinTolerance());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.setPercentOutput(0);
        floor.set(Floor.Speed.STOP);
        shooterAtSpeed = false;
    }
}

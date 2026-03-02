package frc.robot.commands.WCP;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // Interpolation table - adjusted for trajectory and power
        // RPMs increased by ~350-400 total for additional distance
        // Hood angles raised to flatten trajectory (hood mechanism is inverted)
        distanceToShotMap.put(Inches.of(52.0), new Shot(3150, 0.24));  // Was 2800, now +350
        distanceToShotMap.put(Inches.of(114.4), new Shot(3675, 0.48)); // Was 3275, now +400
        distanceToShotMap.put(Inches.of(165.5), new Shot(4050, 0.56)); // Was 3650, now +400
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final Floor floor;
    private final Distance targetDistance;
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
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.floor = floor;
        this.targetDistance = Meters.of(distanceMeters);
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
        final Shot shot = distanceToShotMap.get(targetDistance);
        
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
        
        SmartDashboard.putNumber("Static Distance to Target (inches)", targetDistance.in(Inches));
        SmartDashboard.putNumber("Target Shooter RPM", shot.shooterRPM);
        SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
        SmartDashboard.putBoolean("Shooter At Speed", shooterAtSpeed);
        SmartDashboard.putBoolean("Hood At Position", hood.isPositionWithinTolerance());
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

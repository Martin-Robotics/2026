package frc.robot.commands.WCP;

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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Prepares the shooter and feeds game pieces at a known static distance.
 * Unlike PrepareShotCommand, this does not use field position or odometry.
 * Spins up the shooter to the interpolated RPM and engages the feeder and floor once at speed.
 * 
 * Also gently agitates the intake arm to nudge balls toward the feeder.
 * The arm oscillates between STOWED and AGITATE on a timer while rollers run.
 * 
 * NOTE: Hood positioning has been moved to PrepareToFire (Driver Y button).
 * This command no longer waits for the hood — it fires as soon as the shooter is at speed.
 */
public class PrepareStaticShotCommand extends Command {
    /** Shared interpolation table — also used by PrepareToFire for hood positioning. */
    public static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
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
    private final Feeder feeder;
    private final Floor floor;
    private final Intake intake;
    private final Distance targetDistance;
    private final boolean useDetectedDistance;
    private final frc.robot.subsystems.LimelightSubsystem6237 limelight;
    private boolean shooterAtSpeed = false;
    
    // Agitate state
    private final Timer agitateTimer = new Timer();
    private boolean agitateAtIntake = false;
    private boolean agitateStarted = false;

    /**
     * Creates a command to prepare for a shot at a static distance.
     * 
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor The floor subsystem
     * @param intake The intake subsystem (for agitation during shooting)
     * @param distanceMeters The distance to the target in meters
     */
    public PrepareStaticShotCommand(Shooter shooter, Feeder feeder, Floor floor, Intake intake, double distanceMeters) {
        this(shooter, feeder, floor, intake, distanceMeters, false, null);
    }
    
    /**
     * Creates a command to prepare for a shot, optionally using detected distance from Limelight.
     * 
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor The floor subsystem
     * @param intake The intake subsystem (for agitation during shooting)
     * @param fallbackDistanceMeters The fallback distance if no detected distance available
     * @param useDetectedDistance If true, uses last detected distance from Limelight background tracking
     * @param limelight The Limelight subsystem (required if useDetectedDistance is true)
     */
    public PrepareStaticShotCommand(Shooter shooter, Feeder feeder, Floor floor, Intake intake, double fallbackDistanceMeters, boolean useDetectedDistance, frc.robot.subsystems.LimelightSubsystem6237 limelight) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.floor = floor;
        this.intake = intake;
        this.targetDistance = Meters.of(fallbackDistanceMeters);
        this.useDetectedDistance = useDetectedDistance;
        this.limelight = limelight;
        addRequirements(shooter, feeder, floor, intake);
    }

    @Override
    public void initialize() {
        shooterAtSpeed = false;
        // Start agitate delay timer — agitation begins after kAgitateDelaySeconds
        agitateTimer.restart();
        agitateAtIntake = false;
        agitateStarted = false;
    }

    public boolean isReadyToShoot() {
        return shooterAtSpeed;
    }

    @Override
    public void execute() {
        // ---- Shooter logic ----
        // Determine which distance to use
        Distance actualDistance = targetDistance;
        
        if (useDetectedDistance && limelight != null) {
            // Use hub-center corrected distance from Limelight
            double detectedDistance = limelight.getHubCenterDistance();
            
            if (detectedDistance > 0) {
                actualDistance = Meters.of(detectedDistance);
            }
        }
        
        final Shot shot = distanceToShotMap.get(actualDistance);
        
        // Spin up shooter to target RPM
        shooter.setRPM(shot.shooterRPM);
        
        // Track when shooter reaches speed
        if (!shooterAtSpeed && shooter.isVelocityWithinTolerance()) {
            shooterAtSpeed = true;
        }
        
        // Continuously command feeder and floor once shooter is at speed
        // (Must be outside the !shooterAtSpeed check to keep them running every cycle)
        if (shooterAtSpeed) {
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
        }
        
        // ---- Agitate logic ----
        // Wait for initial delay, then oscillate intake arm between INTAKE and AGITATE
        if (!agitateStarted) {
            // During delay, hold intake at INTAKE position (don't leave motor uncontrolled)
            intake.setManualPosition(Intake.Position.INTAKE);
            if (agitateTimer.hasElapsed(Constants.Intake.kAgitateDelaySeconds)) {
                agitateStarted = true;
                agitateTimer.restart();
                agitateAtIntake = false;
                intake.setManualPosition(Intake.Position.AGITATE);
                double rollerPercent = SmartDashboard.getNumber("Intake/Roller Speed %", Constants.Intake.kIntakePercentOutput);
                intake.setManualRollerVoltage(rollerPercent);
            }
        } else {
            double interval = Constants.Intake.kAgitateIntervalSeconds;
            if (agitateTimer.hasElapsed(interval)) {
                agitateTimer.restart();
                agitateAtIntake = !agitateAtIntake;
                if (agitateAtIntake) {
                    intake.setManualPosition(Intake.Position.INTAKE);
                } else {
                    intake.setManualPosition(Intake.Position.AGITATE);
                }
                double rollerPercent = SmartDashboard.getNumber("Intake/Roller Speed %", Constants.Intake.kIntakePercentOutput);
                intake.setManualRollerVoltage(rollerPercent);
            }
        }
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
        // Return intake to INTAKE position (not stowed) and stop rollers
        intake.setManualPosition(Intake.Position.INTAKE);
        intake.setManualRollerVoltage(0);
        agitateTimer.stop();
    }
}

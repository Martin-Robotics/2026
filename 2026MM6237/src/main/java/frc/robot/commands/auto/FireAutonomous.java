package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.WCP.PrepareStaticShotCommand;
import frc.robot.commands.WCP.PrepareShotCommand.Shot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Autonomous command to fire at the hub.
 * 
 * Uses the SHARED interpolation table from PrepareStaticShotCommand (teleop RT fire):
 * - Reads hub-center-corrected distance from Limelight background tracking
 * - Looks up RPM and hood position from the shared interpolation table
 * - Spins up shooter and positions hood
 * - Once both are ready, engages feeder and floor for a fixed duration
 * - Automatically ends after firing duration completes
 * 
 * Use the optional fireDurationSeconds constructor parameter to create longer-firing
 * variants (e.g., "Fire2" for full hopper dumps).
 * 
 * Also agitates the intake arm (same as teleop) to help feed balls to the shooter.
 */
public class FireAutonomous extends Command {

    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Floor floor;
    private final Intake intake;
    private final LimelightSubsystem6237 limelight;
    private final double fireDurationSeconds;
    
    private final Timer fireTimer = new Timer();
    private boolean shooterAtSpeed = false;

    // Agitate state (matches PrepareStaticShotCommand)
    private final Timer agitateTimer = new Timer();
    private boolean agitateAtIntake = false;
    private boolean agitateStarted = false;

    /**
     * Creates a FireAutonomous command with the default fire duration.
     */
    public FireAutonomous(Feeder feeder, Shooter shooter, Hood hood, Floor floor, Intake intake, LimelightSubsystem6237 limelight) {
        this(feeder, shooter, hood, floor, intake, limelight, Constants.Auto.kAutoFireRunTimeSeconds);
    }

    /**
     * Creates a FireAutonomous command with a custom fire duration.
     * @param fireDurationSeconds How long to run feeder/floor after shooter reaches speed
     */
    public FireAutonomous(Feeder feeder, Shooter shooter, Hood hood, Floor floor, Intake intake, LimelightSubsystem6237 limelight, double fireDurationSeconds) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.floor = floor;
        this.intake = intake;
        this.limelight = limelight;
        this.fireDurationSeconds = fireDurationSeconds;
        addRequirements(feeder, shooter, hood, floor, intake);
    }

    @Override
    public void initialize() {
        fireTimer.reset();
        shooterAtSpeed = false;
        // Reset agitate state
        agitateTimer.restart();
        agitateAtIntake = false;
        agitateStarted = false;
    }

    @Override
    public void execute() {
        // Get hub-center-corrected distance from Limelight background tracking
        // Matches teleop PrepareStaticShotCommand which also uses getHubCenterDistance()
        double detectedDistance = limelight.getHubCenterDistance();
        Distance actualDistance = detectedDistance > 0 
            ? Meters.of(detectedDistance) 
            : Meters.of(3.0); // Fallback to 3m if no hub seen
        
        // Look up RPM and hood from SHARED interpolation table (single source of truth)
        final Shot shot = PrepareStaticShotCommand.distanceToShotMap.get(actualDistance);
        
        // Apply auto-only RPM boost (tunable from SmartDashboard)
        double rpmBoost = SmartDashboard.getNumber("Auto/RPM Boost", Constants.Auto.kAutoRpmBoostDefault);
        double boostedRPM = shot.shooterRPM + rpmBoost;
        
        // Always spin up shooter and position hood
        shooter.setRPM(boostedRPM);
        hood.setPosition(shot.hoodPosition);
        
        if (!shooterAtSpeed) {
            boolean shooterReady = shooter.isVelocityWithinTolerance();
            boolean hoodReady = hood.isPositionWithinTolerance();
            
            if (shooterReady && hoodReady) {
                // Both ready — start firing
                shooterAtSpeed = true;
                fireTimer.start();
                feeder.set(Feeder.Speed.FEED);
                floor.set(Floor.Speed.FEED);
            }
        }

        // ---- Agitate logic (matches PrepareStaticShotCommand) ----
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
        return shooterAtSpeed && fireTimer.hasElapsed(fireDurationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        fireTimer.stop();
        shooter.stop();
        feeder.setPercentOutput(0);
        floor.set(Floor.Speed.STOP);
        // Return intake to INTAKE position and stop rollers (matches PrepareStaticShotCommand)
        intake.setManualPosition(Intake.Position.INTAKE);
        intake.setManualRollerVoltage(0);
        agitateTimer.stop();
    }
}

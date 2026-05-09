package frc.robot.commands.WCP;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * SEND IT — maximum RPM, maximum hood angle, feed immediately at speed.
 *
 * Use this to probe the upper limits of shooter range without exceeding
 * hardware safety limits. Current limits are enforced by TalonFX config
 * (stator: 120 A, supply: 70 A). Requesting RPM above motor free-speed
 * simply runs the motors at full voltage — no mechanical damage risk.
 *
 * Settings:
 *   Shooter: 5500 RPM commanded (→ 6875 RPM adjusted, motors run at ~12 V)
 *   Hood:    0.75 (maximum elevation)
 *   Feed:    starts as soon as shooter is at speed
 *   Agitate: same timing as PrepareStaticShotCommand
 */
public class StaticSendItCommand extends Command {

    // Max RPM to command — after the 1.25x roller-compensation multiplier in
    // Shooter.setRPM() this becomes 6875 RPM, which clamps to full 12 V output.
    private static final double SEND_IT_RPM = 5500;

    // Maximum hood elevation (same upper bound as manual hood-up control).
    private static final double SEND_IT_HOOD = 0.75;

    private final Shooter shooter;
    private final Feeder feeder;
    private final Floor floor;
    private final Hood hood;
    private final Intake intake;

    private boolean shooterAtSpeed = false;

    // Fallback: start feeding after this many seconds even if shooter never reaches speed
    private static final double FEED_TIMEOUT_SECONDS = 2.5;
    private final Timer feedTimeoutTimer = new Timer();

    // Agitate state — mirrors PrepareStaticShotCommand behaviour
    private final Timer agitateTimer = new Timer();
    private boolean agitateAtIntake = false;
    private boolean agitateStarted = false;

    public StaticSendItCommand(Shooter shooter, Feeder feeder, Floor floor, Hood hood, Intake intake) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.floor = floor;
        this.hood = hood;
        this.intake = intake;
        addRequirements(shooter, feeder, floor, hood, intake);
    }

    @Override
    public void initialize() {
        shooterAtSpeed = false;
        feedTimeoutTimer.restart();
        agitateTimer.restart();
        agitateAtIntake = false;
        agitateStarted = false;

        // Set hood to maximum elevation immediately
        hood.setPosition(SEND_IT_HOOD);
    }

    @Override
    public void execute() {
        // Spin up shooter to max
        shooter.setRPM(SEND_IT_RPM);

        // Engage feeder and floor once shooter is at speed, or after 2.5s timeout
        if (!shooterAtSpeed && (shooter.isVelocityWithinTolerance() || feedTimeoutTimer.hasElapsed(FEED_TIMEOUT_SECONDS))) {
            shooterAtSpeed = true;
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
        }

        // Agitate intake to nudge balls toward feeder
        if (!agitateStarted) {
            if (agitateTimer.hasElapsed(Constants.Intake.kAgitateDelaySeconds)) {
                agitateStarted = true;
                agitateTimer.restart();
                agitateAtIntake = false;
                intake.setManualPosition(Intake.Position.AGITATE);
                double rollerPercent = SmartDashboard.getNumber("Intake/Roller Speed %", Constants.Intake.kIntakePercentOutput);
                intake.setManualRollerVoltage(rollerPercent);
            }
        } else {
            if (agitateTimer.hasElapsed(Constants.Intake.kAgitateIntervalSeconds)) {
                agitateTimer.restart();
                agitateAtIntake = !agitateAtIntake;
                intake.setManualPosition(agitateAtIntake ? Intake.Position.INTAKE : Intake.Position.AGITATE);
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
        intake.setManualPosition(Intake.Position.INTAKE);
        intake.setManualRollerVoltage(0);
        agitateTimer.stop();
        feedTimeoutTimer.stop();
    }
}

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
 * Fires balls back toward the alliance starting zone at a fixed speed.
 * 
 * Unlike PrepareStaticShotCommand (RT), this command:
 * - Uses static RPM and hood position from Constants (no Limelight / distance detection)
 * - Starts the feeder immediately without waiting for the shooter to reach full speed
 * - Is intended for returning balls, not scoring into the hub
 * 
 * Also gently agitates the intake arm to nudge balls toward the feeder.
 * 
 * Bound to Operator Left Trigger.
 */
public class ReturnShotCommand extends Command {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Floor floor;
    private final Hood hood;
    private final Intake intake;
    
    private boolean shooterAtSpeed = false;
    
    // Agitate state
    private final Timer agitateTimer = new Timer();
    private boolean agitateAtIntake = false;
    private boolean agitateStarted = false;

    /**
     * Creates a return shot command.
     * 
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor The floor subsystem
     * @param hood The hood subsystem
     * @param intake The intake subsystem (for agitation during shooting)
     */
    public ReturnShotCommand(Shooter shooter, Feeder feeder, Floor floor, Hood hood, Intake intake) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.floor = floor;
        this.hood = hood;
        this.intake = intake;
        addRequirements(shooter, feeder, floor, hood, intake);
    }

    @Override
    public void initialize() {
        // Set hood to the static return position immediately
        hood.setPosition(Constants.Shooter.kReturnShotHoodPosition);
        shooterAtSpeed = false;
        // Start agitate delay timer — agitation begins after kAgitateDelaySeconds
        agitateTimer.restart();
        agitateAtIntake = false;
        agitateStarted = false;
    }

    @Override
    public void execute() {
        // Spin up shooter to static return RPM
        shooter.setRPM(Constants.Shooter.kReturnShotRPM);

        // Engage feeder and floor once shooter is at speed (same as PrepareStaticShotCommand)
        if (!shooterAtSpeed && shooter.isVelocityWithinTolerance()) {
            shooterAtSpeed = true;
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
        }
        
        // ---- Agitate logic ----
        // Wait for initial delay, then oscillate intake arm between INTAKE and AGITATE
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
        return false; // Runs while trigger is held
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

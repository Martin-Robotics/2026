package frc.robot.commands.WCP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/**
 * Fires balls back toward the alliance starting zone at a fixed speed.
 * 
 * Unlike PrepareStaticShotCommand (RT), this command:
 * - Uses static RPM and hood position from Constants (no Limelight / distance detection)
 * - Starts the feeder immediately without waiting for the shooter to reach full speed
 * - Is intended for returning balls, not scoring into the hub
 * 
 * Bound to Operator Left Trigger.
 */
public class ReturnShotCommand extends Command {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Floor floor;
    private final Hood hood;

    /**
     * Creates a return shot command.
     * 
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor The floor subsystem
     * @param hood The hood subsystem
     */
    public ReturnShotCommand(Shooter shooter, Feeder feeder, Floor floor, Hood hood) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.floor = floor;
        this.hood = hood;
        addRequirements(shooter, feeder, floor, hood);
    }

    @Override
    public void initialize() {
        // Set hood to the static return position immediately
        hood.setPosition(Constants.Shooter.kReturnShotHoodPosition);
    }

    @Override
    public void execute() {
        // Spin up shooter to static return RPM
        shooter.setRPM(Constants.Shooter.kReturnShotRPM);

        // Feed immediately — no need to wait for full speed for a lob return
        feeder.set(Feeder.Speed.FEED);
        floor.set(Floor.Speed.FEED);
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
    }
}

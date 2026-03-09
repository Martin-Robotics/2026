package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Speed;

/**
 * Autonomous command to stop the intake rollers without moving the arm.
 * 
 * Sets the intake rollers to STOP and finishes immediately.
 * The arm position is not changed — use this when you want to stop
 * collecting notes but keep the arm where it is.
 */
public class StopIntakeRollers extends Command {
    private final Intake intake;

    public StopIntakeRollers(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.set(Speed.STOP);
    }

    @Override
    public boolean isFinished() {
        // Finishes immediately — rollers stay stopped until started again
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up
    }
}

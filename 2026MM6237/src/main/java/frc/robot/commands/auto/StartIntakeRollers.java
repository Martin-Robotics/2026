package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Speed;

/**
 * Autonomous command to start the intake rollers without moving the arm.
 * 
 * Sets the intake rollers to INTAKE speed and finishes immediately.
 * The arm position is not changed — use this when the arm is already
 * in the desired position and you just need to spin up the rollers.
 * 
 * Note: The Intake subsystem safety will prevent rollers from running
 * if the arm is in the STOWED position.
 */
public class StartIntakeRollers extends Command {
    private final Intake intake;

    public StartIntakeRollers(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.set(Speed.INTAKE);
    }

    @Override
    public boolean isFinished() {
        // Finishes immediately — rollers keep running until stopped
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Rollers continue running; they will be stopped by StopIntakeRollers or StopIntake
    }
}

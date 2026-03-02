package frc.robot.commands.auto;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Speed;
import frc.robot.subsystems.Intake.Position;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Autonomous command to run the intake.
 * 
 * Extends the intake pivot arm to the INTAKE position and spins up the rollers
 * at full speed to collect notes. The arm and rollers continue running when
 * the command ends. This command runs indefinitely until interrupted.
 */
public class RunIntake extends Command {
    private final Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Extend intake arm to INTAKE position
        intake.set(Position.INTAKE);
        // Start intake rollers spinning
        intake.set(Speed.INTAKE);
    }

    @Override
    public void execute() {
        // Rollers continue spinning
    }

    @Override
    public boolean isFinished() {
        // This command runs until interrupted (typically sequenced in an autonomous routine)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Rollers keep spinning even when command ends
        // They will be stopped by another command (e.g., StopIntake)
    }
}

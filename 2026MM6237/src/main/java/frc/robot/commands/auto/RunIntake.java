package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Speed;
import frc.robot.subsystems.Intake.Position;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Autonomous command to run the intake.
 * 
 * Extends the intake pivot arm to the INTAKE position, waits for it to reach
 * the position, then spins up the rollers at full speed to collect notes.
 * The arm and rollers continue running when the command ends.
 */
public class RunIntake extends Command {
    private final Intake intake;
    private boolean armAtPosition = false;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Extend intake arm to INTAKE position
        intake.set(Position.INTAKE);
        armAtPosition = false;
    }

    @Override
    public void execute() {
        // Check if arm has reached the INTAKE position
        if (!armAtPosition) {
            double targetAngle = Position.INTAKE.angle().in(Degrees);
            double currentAngle = intake.getPivotAngleDegrees();
            
            if (Math.abs(currentAngle - targetAngle) < Constants.Intake.kAutoPositionToleranceDegrees) {
                // Arm is at position, now start the rollers
                armAtPosition = true;
                intake.set(Speed.INTAKE);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes once rollers are running
        return armAtPosition;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep arm and rollers running even when command ends
        // They will be stopped by another command (e.g., StopIntake)
    }
}

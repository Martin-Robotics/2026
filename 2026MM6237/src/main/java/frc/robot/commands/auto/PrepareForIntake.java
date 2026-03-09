package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;

/**
 * Autonomous command to prepare the robot for intake.
 * 
 * Moves the intake arm into the intake position to prepare for collecting notes.
 * The command completes once the arm reaches the target position.
 */
public class PrepareForIntake extends Command {
    private final Intake intake;

    public PrepareForIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Move intake arm to INTAKE position
        intake.set(Position.INTAKE);
    }

    @Override
    public void execute() {
        // Wait for arm to reach position
    }

    @Override
    public boolean isFinished() {
        // Check if the pivot angle is within tolerance of the target position
        double targetAngle = Position.INTAKE.angle().in(Degrees);
        double currentAngle = intake.getPivotAngleDegrees();
        return Math.abs(currentAngle - targetAngle) < Constants.Intake.kAutoPositionToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep arm in position
    }
}

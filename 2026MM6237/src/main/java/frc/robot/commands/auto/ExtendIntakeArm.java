package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;

/**
 * Autonomous command to extend the intake arm to the INTAKE position.
 * 
 * Moves ONLY the pivot arm — does NOT start the intake rollers.
 * Use this when you need the arm out of the way but don't want to run rollers
 * (e.g., positioning before a pickup, or clearing the arm before shooting).
 * 
 * Finishes when the arm reaches the INTAKE position within tolerance.
 */
public class ExtendIntakeArm extends Command {
    private final Intake intake;

    public ExtendIntakeArm(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Move arm to INTAKE position (no rollers)
        intake.setManualPosition(Position.INTAKE);
    }

    @Override
    public void execute() {
        // Arm is moving via setManualPosition - just wait
    }

    @Override
    public boolean isFinished() {
        double targetAngle = Position.INTAKE.angle().in(Degrees);
        double currentAngle = intake.getPivotAngleDegrees();
        return Math.abs(currentAngle - targetAngle) < Constants.Intake.kAutoPositionToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep arm at INTAKE position — do NOT start rollers
    }
}

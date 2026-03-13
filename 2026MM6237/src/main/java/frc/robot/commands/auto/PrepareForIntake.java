package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;

/**
 * Autonomous command to prepare the robot for intake.
 * 
 * Moves the intake arm into the intake position to prepare for collecting notes.
 * Includes a safety timeout to prevent hanging the auto sequence.
 */
public class PrepareForIntake extends Command {
    private final Intake intake;
    private static final double kTimeoutSeconds = 2.0;
    private final Timer timer = new Timer();

    public PrepareForIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setManualPosition(Position.INTAKE);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(kTimeoutSeconds)) {
            System.out.println("PrepareForIntake: timeout reached, finishing");
            return true;
        }
        double targetAngle = Position.INTAKE.angle().in(Degrees);
        double currentAngle = intake.getPivotAngleDegrees();
        return Math.abs(currentAngle - targetAngle) < Constants.Intake.kAutoPositionToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}

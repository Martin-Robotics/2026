package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;
import frc.robot.subsystems.Intake.Speed;

/**
 * Autonomous command to stop the intake and retract the arm.
 * 
 * Stops the intake rollers and retracts the intake arm back to the stowed position.
 * The command completes once the arm reaches the stowed position.
 */
public class StopIntake extends Command {
    private final Intake intake;

    public StopIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Stop rollers and retract arm
        intake.set(Speed.STOP);
        intake.set(Position.STOWED);
    }

    @Override
    public void execute() {
        // Wait for arm to reach stowed position
    }

    @Override
    public boolean isFinished() {
        // Check if the pivot angle is within tolerance of the stowed position
        double targetAngle = Position.STOWED.angle().in(Degrees);
        double currentAngle = intake.getPivotAngleDegrees();
        return Math.abs(currentAngle - targetAngle) < Constants.Intake.kAutoPositionToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep arm in stowed position
    }
}

package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;

/**
 * Autonomous command to retract the intake arm to the STOWED position.
 * 
 * Moves ONLY the pivot arm — does NOT stop the intake rollers.
 * If rollers are running from a previous command, they will continue.
 * Use StopIntake if you need to stop rollers AND retract.
 * 
 * Finishes when the arm reaches the STOWED position within tolerance.
 */
public class RetractIntakeArm extends Command {
    private final Intake intake;

    public RetractIntakeArm(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Move arm to STOWED position (no roller changes)
        intake.setManualPosition(Position.STOWED);
    }

    @Override
    public void execute() {
        // Arm is moving via setManualPosition - just wait
    }

    @Override
    public boolean isFinished() {
        double targetAngle = Position.STOWED.angle().in(Degrees);
        double currentAngle = intake.getPivotAngleDegrees();
        return Math.abs(currentAngle - targetAngle) < Constants.Intake.kAutoPositionToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep arm at STOWED position
    }
}

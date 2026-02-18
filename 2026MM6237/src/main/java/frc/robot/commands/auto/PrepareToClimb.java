package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hanger.Position;

/**
 * Autonomous command to prepare the robot for climbing.
 * 
 * Makes fine adjustments to the hanger mechanism to prepare the robot
 * to climb the tower. Moves the hanger to the EXTEND_HOPPER position.
 */
public class PrepareToClimb extends Command {
    private final Hanger hanger;

    public PrepareToClimb(Hanger hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }

    @Override
    public void initialize() {
        // Move hanger to prepare for climbing
        hanger.set(Position.EXTEND_HOPPER);
    }

    @Override
    public void execute() {
        // Wait for hanger to reach position
    }

    @Override
    public boolean isFinished() {
        // Check if the hanger extension is within tolerance of the target position
        double targetExtension = Position.EXTEND_HOPPER.motorAngle().in(edu.wpi.first.units.Units.Rotations) 
            * Constants.Hanger.kAutoRotationsToInchesMultiplier;
        double currentExtension = hanger.getCurrentExtensionInches();
        return Math.abs(currentExtension - targetExtension) < Constants.Hanger.kAutoExtensionToleranceInches;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep hanger in position
    }
}

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hanger.Position;

/**
 * Autonomous command to climb the tower.
 * 
 * Engages the hanger/climb motors to fully climb and hang on the tower.
 * Moves the hanger to the HUNG position to complete the climb.
 * The command runs indefinitely until interrupted.
 */
public class Climb extends Command {
    private final Hanger hanger;

    public Climb(Hanger hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }

    @Override
    public void initialize() {
        // Begin climbing by moving hanger to HANGING position first
        hanger.set(Position.HANGING);
    }

    @Override
    public void execute() {
        // Check if we've reached the hanging position, then move to hung position
        if (isAtPosition(Position.HANGING)) {
            hanger.set(Position.HUNG);
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we've reached the final HUNG position
        return isAtPosition(Position.HUNG);
    }

    @Override
    public void end(boolean interrupted) {
        // Hold the HUNG position
        hanger.set(Position.HUNG);
    }

    /**
     * Checks if the hanger is at the target position within tolerance.
     * 
     * @param targetPosition The target position to check against
     * @return true if hanger is within tolerance of the target position
     */
    private boolean isAtPosition(Position targetPosition) {
        double targetExtension = targetPosition.motorAngle().in(edu.wpi.first.units.Units.Rotations) 
            * Constants.Hanger.kAutoRotationsToInchesMultiplier;
        double currentExtension = hanger.getCurrentExtensionInches();
        return Math.abs(currentExtension - targetExtension) < Constants.Hanger.kAutoExtensionToleranceInches;
    }
}

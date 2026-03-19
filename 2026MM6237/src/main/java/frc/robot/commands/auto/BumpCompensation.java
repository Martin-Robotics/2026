package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Instant command that nudges the robot's odometry pose to compensate for
 * distance lost when crossing a field bump.
 * 
 * PROBLEM: When the robot drives over a bump, wheel encoders count rotations
 * that went into climbing vertically (or wheel slip on the bump surface).
 * The odometry over-counts horizontal distance, so after the bump the robot
 * "thinks" it's further along than it actually is.
 * 
 * FIX: After each bump-crossing path, this command pulls the odometry pose
 * BACK by a tunable amount in the direction the robot just traveled.
 * This makes the next path segment start from a corrected position.
 * 
 * The compensation amount is read from SmartDashboard ("Auto/Bump Comp (m)")
 * so it can be tuned live at competition without redeploying.
 * 
 * USAGE: Insert as a NamedCommand between paths in PathPlanner autos:
 *   [path: SR4 Start To Center]   -- crosses bump going +X
 *   [named: BumpCompOutbound]     -- pulls odometry back in -X
 *   [named: RunIntake]
 *   ...
 *   [path: SR4 Center To Hub]     -- crosses bump going -X
 *   [named: BumpCompReturn]       -- pulls odometry back in +X
 */
public class BumpCompensation extends InstantCommand {

    /** Default bump compensation in meters (positive value = how far to pull back). */
    public static final double kDefaultBumpCompMeters = 0.15;  // ~6 inches — tune on field

    private final CommandSwerveDrivetrain drivetrain;
    private final boolean outbound; // true = robot traveled +X (away from alliance wall)

    /**
     * @param drivetrain The swerve drivetrain
     * @param outbound   true if robot just traveled AWAY from alliance wall (+X for Blue),
     *                   false if robot just traveled BACK toward alliance wall (-X for Blue)
     */
    public BumpCompensation(CommandSwerveDrivetrain drivetrain, boolean outbound) {
        this.drivetrain = drivetrain;
        this.outbound = outbound;
        // No subsystem requirement — this is an instant pose reset, not a drive command
    }

    @Override
    public void initialize() {
        double comp = SmartDashboard.getNumber("Auto/Bump Comp (m)", kDefaultBumpCompMeters);
        comp = Math.max(0.0, Math.min(0.5, comp)); // Clamp 0–0.5m for safety

        if (comp == 0.0) return; // No compensation

        Pose2d currentPose = drivetrain.getState().Pose;

        // The odometry over-counted distance in the travel direction.
        // Pull the pose BACK toward where the robot actually is.
        //
        // Outbound (+X travel): odometry X is too high → subtract comp from X
        // Return   (-X travel): odometry X is too low (went too far in -X) → add comp to X
        double correctedX;
        if (outbound) {
            correctedX = currentPose.getX() - comp;
        } else {
            correctedX = currentPose.getX() + comp;
        }

        Pose2d correctedPose = new Pose2d(
            correctedX,
            currentPose.getY(),
            currentPose.getRotation()
        );

        drivetrain.resetPose(correctedPose);

        System.out.printf("[BumpComp] %s: Adjusted X by %.3fm  (%.2f → %.2f)%n",
            outbound ? "Outbound" : "Return",
            outbound ? -comp : comp,
            currentPose.getX(), correctedX);
    }
}

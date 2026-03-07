package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.commands.WCP.Landmarks;

/**
 * Command to prepare the robot to fire by aiming at the hub.
 * 
 * Uses the WCP-style FieldCentricFacingAngle approach:
 * - Calculates hub direction from field coordinates (using Landmarks.hubPosition())
 * - Uses CTRE's built-in heading PID controller for smooth, stable rotation
 * - Maintains aim even if Limelight temporarily loses the target
 * 
 * The Limelight is used for:
 * - Confirming we're pointed at the right target
 * - Providing accurate distance measurements for shooter RPM calculations
 * 
 * Does NOT spin up shooter - only aims the robot for the Fire command.
 */
public class PrepareToFire extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    
    // Use FieldCentricFacingAngle - CTRE's built-in heading PID (same as WCP AimAndDriveCommand)
    private final SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Constants.Driving.kPIDRotationDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);  // P=5 works well for snappy response without oscillation
    
    // Aim tolerance - how close we need to be to consider "aimed"
    private static final double AIM_TOLERANCE_DEGREES = 3.0;

    public PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        
        // Require drivetrain so we can control rotation
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("PrepareToFire/Status", "Aiming started");
        SmartDashboard.putBoolean("PrepareToFire/Aimed", false);
    }

    /**
     * Calculates the direction from the robot to the hub in operator perspective.
     * This is the same approach used by WCP's AimAndDriveCommand.
     * 
     * Key insight: The swerve pose is in Blue Alliance perspective (field coordinates).
     * We need to convert to Operator perspective for FieldCentricFacingAngle.
     */
    private Rotation2d getDirectionToHub() {
        // Get hub position based on alliance (from WCP Landmarks)
        final Translation2d hubPosition = Landmarks.hubPosition();
        
        // Get robot position from odometry (in Blue Alliance / field coordinates)
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        
        // Calculate direction from robot to hub (in Blue Alliance perspective)
        // hubPosition - robotPosition gives vector pointing toward hub
        // .getAngle() returns the angle of that vector
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        
        // Convert to Operator perspective (what the driver sees)
        // getOperatorForwardDirection() returns the rotation needed to convert
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective
            .rotateBy(drivetrain.getOperatorForwardDirection());
        
        return hubDirectionInOperatorPerspective;
    }

    /**
     * Checks if the robot is currently aimed at the hub within tolerance.
     */
    public boolean isAimed() {
        final Rotation2d targetHeading = getDirectionToHub();
        final Rotation2d currentHeadingInBlueAlliancePerspective = drivetrain.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective
            .rotateBy(drivetrain.getOperatorForwardDirection());
        
        double errorDegrees = Math.abs(targetHeading.minus(currentHeadingInOperatorPerspective).getDegrees());
        return errorDegrees <= AIM_TOLERANCE_DEGREES;
    }

    @Override
    public void execute() {
        // Calculate the target direction to the hub
        Rotation2d targetDirection = getDirectionToHub();
        
        // Get current robot state for debugging
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        Translation2d hubPosition = Landmarks.hubPosition();
        
        // Calculate distance to hub from odometry
        double distanceToHub = robotPosition.getDistance(hubPosition);
        
        // Check if Limelight sees the hub (for confirmation and fine distance)
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        double limelightDistance = limelight.getLastHubDistance();
        int tagID = limelight.getLastHubTagID();
        double tx = limelight.getLastHubTx();
        
        // Use Limelight distance if available, otherwise use odometry
        double reportedDistance = (hubVisible && limelightDistance > 0) ? limelightDistance : distanceToHub;
        String distanceSource = (hubVisible && limelightDistance > 0) ? "Limelight" : "Odometry";
        
        // Check if we're aimed
        boolean aimed = isAimed();
        
        // Update SmartDashboard
        SmartDashboard.putNumber("PrepareToFire/Target Heading (deg)", targetDirection.getDegrees());
        SmartDashboard.putNumber("PrepareToFire/Current Heading (deg)", currentRotation.getDegrees());
        SmartDashboard.putNumber("PrepareToFire/Distance To Hub (m)", reportedDistance);
        SmartDashboard.putString("PrepareToFire/Distance Source", distanceSource);
        SmartDashboard.putBoolean("PrepareToFire/Hub Visible", hubVisible);
        SmartDashboard.putBoolean("PrepareToFire/Aimed", aimed);
        
        if (hubVisible) {
            SmartDashboard.putNumber("PrepareToFire/Limelight TX (deg)", tx);
            SmartDashboard.putNumber("PrepareToFire/Hub Tag ID", tagID);
            SmartDashboard.putString("PrepareToFire/Status", aimed ? "AIMED at Tag " + tagID : "Aiming at Tag " + tagID);
        } else {
            SmartDashboard.putString("PrepareToFire/Status", aimed ? "AIMED (using odometry)" : "Aiming (using odometry)");
        }
        
        // Apply the aim request - CTRE's FieldCentricFacingAngle handles all the rotation PID
        // No translation (VelocityX=0, VelocityY=0), only rotation to face the hub
        drivetrain.setControl(
            aimRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(targetDirection)
        );
    }

    @Override
    public boolean isFinished() {
        // Never finish automatically - operator holds button to aim and read distance
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(
            aimRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(drivetrain.getState().Pose.getRotation()) // Hold current heading
        );
        SmartDashboard.putString("PrepareToFire/Status", "Command ended");
        SmartDashboard.putBoolean("PrepareToFire/Aimed", false);
    }
}

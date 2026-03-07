package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.commands.WCP.Landmarks;

/**
 * Autonomous command to prepare the robot to fire by aiming at the hub.
 * 
 * Similar to PrepareToFire but designed for autonomous use:
 * - Attempts to aim at the hub for a fixed duration (kAutoPrepareAimTimeSeconds)
 * - Automatically ends after the timeout OR when properly aimed
 * - Does NOT require controller input
 * 
 * Uses the WCP-style FieldCentricFacingAngle approach:
 * - Calculates hub direction from field coordinates (using Landmarks.hubPosition())
 * - Uses CTRE's built-in heading PID controller for smooth, stable rotation
 * 
 * The Limelight is used for:
 * - Confirming we're pointed at the right target
 * - Providing accurate distance measurements for shooter RPM calculations
 * 
 * Does NOT spin up shooter - only aims the robot.
 */
public class PrepareToFireAutonomous extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Use FieldCentricFacingAngle - CTRE's built-in heading PID (same as WCP AimAndDriveCommand)
    private final SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Constants.Driving.kPIDRotationDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);  // P=5 works well for snappy response without oscillation
    
    // Aim tolerance - how close we need to be to consider "aimed"
    private static final double AIM_TOLERANCE_DEGREES = 3.0;
    
    private final Timer aimTimer = new Timer();

    public PrepareToFireAutonomous(LimelightSubsystem6237 limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        
        // Require drivetrain so we can control rotation
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        aimTimer.reset();
        aimTimer.start();
        SmartDashboard.putString("PrepareToFireAuto/Status", "Aiming started");
        SmartDashboard.putBoolean("PrepareToFireAuto/Aimed", false);
    }

    /**
     * Calculates the direction from the robot to the hub in operator perspective.
     * This is the same approach used by WCP's AimAndDriveCommand.
     */
    private Rotation2d getDirectionToHub() {
        // Get hub position based on alliance (from WCP Landmarks)
        final Translation2d hubPosition = Landmarks.hubPosition();
        
        // Get robot position from odometry (in Blue Alliance / field coordinates)
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        
        // Calculate direction from robot to hub (in Blue Alliance perspective)
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        
        // Convert to Operator perspective
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
        
        // Use Limelight distance if available, otherwise use odometry
        double reportedDistance = (hubVisible && limelightDistance > 0) ? limelightDistance : distanceToHub;
        String distanceSource = (hubVisible && limelightDistance > 0) ? "Limelight" : "Odometry";
        
        // Check if we're aimed
        boolean aimed = isAimed();
        
        // Show time remaining
        double timeRemaining = Constants.Auto.kAutoPrepareAimTimeSeconds - aimTimer.get();
        
        // Update SmartDashboard
        SmartDashboard.putNumber("PrepareToFireAuto/Target Heading (deg)", targetDirection.getDegrees());
        SmartDashboard.putNumber("PrepareToFireAuto/Current Heading (deg)", currentRotation.getDegrees());
        SmartDashboard.putNumber("PrepareToFireAuto/Distance To Hub (m)", reportedDistance);
        SmartDashboard.putString("PrepareToFireAuto/Distance Source", distanceSource);
        SmartDashboard.putBoolean("PrepareToFireAuto/Hub Visible", hubVisible);
        SmartDashboard.putBoolean("PrepareToFireAuto/Aimed", aimed);
        SmartDashboard.putNumber("PrepareToFireAuto/Time Remaining", timeRemaining);
        
        if (aimed) {
            SmartDashboard.putString("PrepareToFireAuto/Status", "AIMED!");
        } else {
            SmartDashboard.putString("PrepareToFireAuto/Status", "Aiming...");
        }
        
        // Apply the aim request - CTRE's FieldCentricFacingAngle handles all the rotation PID
        drivetrain.setControl(
            aimRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(targetDirection)
        );
    }

    @Override
    public boolean isFinished() {
        // End when aimed OR when timeout expires
        boolean timedOut = aimTimer.hasElapsed(Constants.Auto.kAutoPrepareAimTimeSeconds);
        boolean aimed = isAimed();
        
        if (timedOut && !aimed) {
            SmartDashboard.putString("PrepareToFireAuto/Status", "Timeout - not fully aimed");
        }
        
        return aimed || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        aimTimer.stop();
        
        // Stop rotation but hold current heading
        drivetrain.setControl(
            aimRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(drivetrain.getState().Pose.getRotation())
        );
        
        if (interrupted) {
            SmartDashboard.putString("PrepareToFireAuto/Status", "Interrupted");
        } else if (isAimed()) {
            SmartDashboard.putString("PrepareToFireAuto/Status", "Complete - AIMED");
        } else {
            SmartDashboard.putString("PrepareToFireAuto/Status", "Complete - timeout");
        }
    }
}

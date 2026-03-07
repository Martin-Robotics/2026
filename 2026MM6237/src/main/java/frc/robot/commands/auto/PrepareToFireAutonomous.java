package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to prepare the robot to fire by aiming at the hub AprilTag.
 * 
 * Uses DIRECT Limelight TX feedback to rotate toward the target:
 * - When Limelight sees the hub tag, uses TX to calculate rotation needed
 * - Maintains last known target heading if tag is temporarily lost
 * - Automatically ends when aimed OR when timeout expires
 * 
 * This approach works regardless of odometry accuracy or field setup.
 */
public class PrepareToFireAutonomous extends Command {
    private final LimelightSubsystem6237 limelight;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Simple field-centric request for manual rotation control
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // Rotation control - tuned for smooth aiming
    private static final double PROPORTIONAL_GAIN = 0.05;
    private static final double MIN_ROTATION_SPEED = 0.3;
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    
    private final Timer aimTimer = new Timer();
    private Rotation2d lastTargetHeading = null;
    private boolean hasEverSeenTarget = false;

    public PrepareToFireAutonomous(LimelightSubsystem6237 limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        aimTimer.reset();
        aimTimer.start();
        lastTargetHeading = null;
        hasEverSeenTarget = false;
        SmartDashboard.putString("PrepareToFireAuto/Status", "Searching for hub...");
        SmartDashboard.putBoolean("PrepareToFireAuto/Aimed", false);
    }

    /**
     * Checks if currently aimed (TX within tolerance).
     */
    public boolean isAimed() {
        if (!limelight.isHubCurrentlyVisible()) {
            return false;
        }
        return Math.abs(limelight.getLastHubTx()) <= AIM_TOLERANCE_DEGREES;
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
        
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        double tx = limelight.getLastHubTx();
        double distance = limelight.getLastHubDistance();
        int tagID = limelight.getLastHubTagID();
        
        double rotationalRate = 0;
        
        if (hubVisible) {
            hasEverSeenTarget = true;
            lastTargetHeading = currentHeading.plus(Rotation2d.fromDegrees(tx));
            
            if (Math.abs(tx) > AIM_TOLERANCE_DEGREES) {
                rotationalRate = -tx * PROPORTIONAL_GAIN;
                
                if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                    rotationalRate = MIN_ROTATION_SPEED;
                } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                    rotationalRate = -MIN_ROTATION_SPEED;
                }
            }
            
            SmartDashboard.putNumber("PrepareToFireAuto/TX (deg)", tx);
            SmartDashboard.putNumber("PrepareToFireAuto/Hub Tag ID", tagID);
            SmartDashboard.putString("PrepareToFireAuto/Status", 
                Math.abs(tx) <= AIM_TOLERANCE_DEGREES ? "AIMED!" : "Aiming...");
            SmartDashboard.putBoolean("PrepareToFireAuto/Aimed", Math.abs(tx) <= AIM_TOLERANCE_DEGREES);
            
        } else if (hasEverSeenTarget && lastTargetHeading != null) {
            Rotation2d error = lastTargetHeading.minus(currentHeading);
            double errorDegrees = error.getDegrees();
            
            if (Math.abs(errorDegrees) > AIM_TOLERANCE_DEGREES) {
                rotationalRate = errorDegrees * PROPORTIONAL_GAIN;
                
                if (rotationalRate > 0 && rotationalRate < MIN_ROTATION_SPEED) {
                    rotationalRate = MIN_ROTATION_SPEED;
                } else if (rotationalRate < 0 && rotationalRate > -MIN_ROTATION_SPEED) {
                    rotationalRate = -MIN_ROTATION_SPEED;
                }
            }
            
            SmartDashboard.putString("PrepareToFireAuto/Status", "Target lost - holding");
            SmartDashboard.putBoolean("PrepareToFireAuto/Aimed", false);
        } else {
            SmartDashboard.putString("PrepareToFireAuto/Status", "Searching...");
            SmartDashboard.putBoolean("PrepareToFireAuto/Aimed", false);
        }
        
        rotationalRate = Math.max(-Constants.TempSwerve.MaxAngularRate, 
                        Math.min(Constants.TempSwerve.MaxAngularRate, rotationalRate));
        
        double timeRemaining = Constants.Auto.kAutoPrepareAimTimeSeconds - aimTimer.get();
        SmartDashboard.putNumber("PrepareToFireAuto/Time Remaining", timeRemaining);
        SmartDashboard.putBoolean("PrepareToFireAuto/Hub Visible", hubVisible);
        SmartDashboard.putNumber("PrepareToFireAuto/Distance To Hub (m)", distance > 0 ? distance : -1);
        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = aimTimer.hasElapsed(Constants.Auto.kAutoPrepareAimTimeSeconds);
        boolean aimed = isAimed();
        return aimed || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        aimTimer.stop();
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
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

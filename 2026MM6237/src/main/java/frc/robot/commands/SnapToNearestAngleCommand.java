// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that rotates the robot to the nearest 45-degree increment.
 * When activated, it calculates the current heading and snaps to the closest
 * of the 8 cardinal/diagonal directions (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°).
 */
public class SnapToNearestAngleCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private Rotation2d targetRotation;
    private final SwerveRequest.FieldCentric driveRequest;
    
    // Only the 4 diagonal angles
    private static final double[] SNAP_ANGLES = {45.0, 135.0, 225.0, 315.0};

    /**
     * Creates a SnapToNearestAngleCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     */
    public SnapToNearestAngleCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        // Create a field-centric request for applying direct rotational rates
        this.driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
            .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        addRequirements(drivetrain);
    }

    /**
     * Calculate the nearest diagonal angle (45°, 135°, 225°, or 315°) to the given angle.
     *
     * @param currentAngle The current angle in degrees
     * @return The nearest diagonal angle
     */
    private static double snapToNearestDiagonal(double currentAngle) {
        // Normalize angle to 0-360 range
        currentAngle = currentAngle % 360.0;
        if (currentAngle < 0) {
            currentAngle += 360.0;
        }
        
        // Find the closest diagonal angle
        double closestAngle = SNAP_ANGLES[0];
        double smallestDifference = Math.abs(angleDifference(currentAngle, SNAP_ANGLES[0]));
        
        for (int i = 1; i < SNAP_ANGLES.length; i++) {
            double difference = Math.abs(angleDifference(currentAngle, SNAP_ANGLES[i]));
            if (difference < smallestDifference) {
                smallestDifference = difference;
                closestAngle = SNAP_ANGLES[i];
            }
        }
        
        return closestAngle;
    }
    
    /**
     * Calculate the shortest angular difference between two angles.
     * 
     * @param angle1 First angle in degrees
     * @param angle2 Second angle in degrees
     * @return Shortest difference in degrees (positive or negative)
     */
    private static double angleDifference(double angle1, double angle2) {
        double diff = angle2 - angle1;
        // Normalize to -180 to 180 range
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    @Override
    public void initialize() {
        // Get current robot rotation
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        double currentDegrees = currentRotation.getDegrees();
        
        // Calculate nearest diagonal angle (45°, 135°, 225°, or 315°)
        double targetDegrees = snapToNearestDiagonal(currentDegrees);
        targetRotation = Rotation2d.fromDegrees(targetDegrees);
    }

    @Override
    public void execute() {
        // Get current robot rotation
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        
        // Calculate the shortest rotation error
        Rotation2d rotationError = targetRotation.minus(currentRotation);
        double errorRadians = rotationError.getRadians();
        
        // Calculate desired rotational rate with proportional control
        // Scale the error by a gain to convert angle error to rotation speed
        double proportionalGain = 1.5; // rad/s per radian of error
        double rotationalRate = errorRadians * proportionalGain;
        
        // Add a minimum rotation speed to overcome deadband when there's significant error
        double minRotationSpeed = 0.8; // rad/s minimum when error > threshold
        double errorThreshold = 0.1; // ~5.7 degrees
        
        if (Math.abs(errorRadians) > errorThreshold) {
            // Apply minimum speed in the direction needed
            if (rotationalRate > 0 && rotationalRate < minRotationSpeed) {
                rotationalRate = minRotationSpeed;
            } else if (rotationalRate < 0 && rotationalRate > -minRotationSpeed) {
                rotationalRate = -minRotationSpeed;
            }
        } else {
            // Near the target, stop rotating
            rotationalRate = 0;
        }
        
        // Clamp to maximum angular rate
        rotationalRate = Math.max(
            -Constants.TempSwerve.MaxAngularRate,
            Math.min(Constants.TempSwerve.MaxAngularRate, rotationalRate)
        );
        
        // Apply the command with no translation, only rotation
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when command ends
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        // Command runs indefinitely while button is held
        return false;
    }
}

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
 * Command that rotates the robot to face a specific direction relative to the field.
 * The robot maintains the facing direction while the command is held.
 */
public class FaceDirectionCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Rotation2d targetRotation;
    private final SwerveRequest.FieldCentric driveRequest;

    /**
     * Creates a FaceDirectionCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param targetRotation The target rotation to face (0° = forward, 90° = left, 180° = back, 270° = right)
     */
    public FaceDirectionCommand(CommandSwerveDrivetrain drivetrain, Rotation2d targetRotation) {
        this.drivetrain = drivetrain;
        this.targetRotation = targetRotation;
        
        // Create a field-centric request for applying direct rotational rates
        this.driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.TempSwerve.MaxSpeed * Constants.OperatorConstants.driverStickDeadband)
            .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        addRequirements(drivetrain);
    }

    /**
     * Creates a FaceDirectionCommand for a specific direction.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param direction The direction to face: "forward", "left", "backward", "right", or "operator"
     */
    public FaceDirectionCommand(CommandSwerveDrivetrain drivetrain, String direction) {
        this(drivetrain, getRotationForDirection(direction));
    }

    /**
     * Helper method to convert direction strings to Rotation2d angles.
     * Assuming the field is oriented with:
     * - 0° = Forward (toward red alliance wall)
     * - 90° = Left wall
     * - 180° = Backward (toward blue alliance wall)
     * - 270°/−90° = Right wall
     * - 180° = Operator (assuming operator is at blue alliance wall)
     *
     * @param direction The direction string ("forward", "left", "backward", "right", "operator")
     * @return The corresponding Rotation2d angle
     */
    private static Rotation2d getRotationForDirection(String direction) {
        return switch (direction.toLowerCase()) {
            case "forward" -> Rotation2d.kZero; // Y button: face forward (0°)
            case "left" -> Rotation2d.fromDegrees(90); // X button: face left wall (90°)
            case "backward" -> Rotation2d.k180deg; // A button: face operator/backward (180°)
            case "right" -> Rotation2d.fromDegrees(270); // B button: face right wall (270°)
            case "operator" -> Rotation2d.k180deg; // Alternative name for backward
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    @Override
    public void initialize() {
        // Command starts - will continuously try to face the target direction
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
        double proportionalGain = 3.0; // rad/s per radian of error
        double rotationalRate = errorRadians * proportionalGain;
        
        // Add a minimum rotation speed to overcome deadband when there's significant error
        double minRotationSpeed = 1.5; // rad/s minimum when error > threshold
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
        // The mapXboxController will handle the button release
        return false;
    }
}


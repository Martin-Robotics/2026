package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FaceDirectionCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriverController {
    
    public static double invertXNumberFieldCentric = 1.0;
    public static double invertYNumberFieldCentric = 1.0;

    public static double invertXNumberRobotCentric = 1.0;
    public static double invertYNumberRobotCentric = 1.0;

    public static Trigger robotCentricControl;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband).withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, NetworkTable limelight) {
        robotCentricControl = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold);

        // Command defaultDrivetrainCommand = drivetrain.applyRequest(() -> {
        //     // Get joystick inputs
        //     double velocityX = driverController.getLeftY() * Constants.TempSwerve.MaxSpeed;
        //     double velocityY = driverController.getLeftX() * Constants.TempSwerve.MaxSpeed;
        //     double rotationalRate = -1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate;
            
        //     // If no joystick input, return idle to prevent motor seeking
        //     if (Math.abs(velocityX) < 0.01 && Math.abs(velocityY) < 0.01 && Math.abs(rotationalRate) < 0.01) {
        //         return new SwerveRequest.Idle();
        //     }
            
        //     if (robotCentricControl.getAsBoolean()) {
        //         // Robot-centric control when left trigger is pressed
        //         return robotCentricDrive
        //             .withVelocityX(invertXNumberRobotCentric * velocityX)
        //             .withVelocityY(invertYNumberRobotCentric * velocityY)
        //             .withRotationalRate(rotationalRate);
        //     } else {
        //         // Field-centric control (default)
        //         return drive
        //             .withVelocityX(invertXNumberFieldCentric * velocityX)
        //             .withVelocityY(invertYNumberFieldCentric * velocityY)
        //             .withRotationalRate(rotationalRate);
        //     }
        // });
        //         Command defaultDrivetrainCommand = drivetrain.applyRequest(() -> {
        //     // Get raw joystick inputs
        //     double rawVelocityX = driverController.getLeftY();
        //     double rawVelocityY = driverController.getLeftX();
        //     double rawRotationalRate = -1 * driverController.getRightX();
            
        //     // Apply deadband to raw inputs first
        //     double deadband = OperatorConstants.driverStickDeadband;
        //     if (Math.abs(rawVelocityX) < deadband) rawVelocityX = 0;
        //     if (Math.abs(rawVelocityY) < deadband) rawVelocityY = 0;
        //     if (Math.abs(rawRotationalRate) < deadband) rawRotationalRate = 0;
            
        //     // If no joystick input after deadband, return idle
        //     if (rawVelocityX == 0 && rawVelocityY == 0 && rawRotationalRate == 0) {
        //         return new SwerveRequest.Idle();
        //     }
            
        //     // Now scale by max values
        //     double velocityX = rawVelocityX * Constants.TempSwerve.MaxSpeed;
        //     double velocityY = rawVelocityY * Constants.TempSwerve.MaxSpeed;
        //     double rotationalRate = rawRotationalRate * Constants.TempSwerve.MaxAngularRate;
            
        //     if (robotCentricControl.getAsBoolean()) {
        //         // Robot-centric control when left trigger is pressed
        //         return new SwerveRequest.RobotCentric()
        //             .withVelocityX(invertXNumberRobotCentric * velocityX)
        //             .withVelocityY(invertYNumberRobotCentric * velocityY)
        //             .withRotationalRate(rotationalRate)
        //             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        //     } else {
        //         // Field-centric control (default)
        //         return new SwerveRequest.FieldCentric()
        //             .withVelocityX(invertXNumberFieldCentric * velocityX)
        //             .withVelocityY(invertYNumberFieldCentric * velocityY)
        //             .withRotationalRate(rotationalRate)
        //             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        //     }
        // });

        Command defaultDrivetrainCommand = drivetrain.applyRequest(() -> {
            // Apply deadband to joystick inputs first
            double leftY = applyDeadband(-driverController.getLeftY(), OperatorConstants.driverStickDeadband);
            double leftX = applyDeadband(-driverController.getLeftX(), OperatorConstants.driverStickDeadband);
            double rightX = applyDeadband(-driverController.getRightX(), OperatorConstants.driverStickDeadband);
            
            return drive
                .withVelocityX(leftY * Constants.TempSwerve.MaxSpeed)
                .withVelocityY(leftX * Constants.TempSwerve.MaxSpeed)
                .withRotationalRate(rightX * Constants.TempSwerve.MaxAngularRate);
        });

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            defaultDrivetrainCommand
        );

        // Map face buttons to face specific directions
        // Y button: Face forward (toward red alliance wall)
        driverController.y().whileTrue(new FaceDirectionCommand(drivetrain, "forward"));
        
        // X button: Face left wall
        driverController.x().whileTrue(new FaceDirectionCommand(drivetrain, "left"));
        
        // B button: Face right wall
        driverController.b().whileTrue(new FaceDirectionCommand(drivetrain, "right"));
        
        // A button: Face operator/backward (toward blue alliance wall)
        driverController.a().whileTrue(new FaceDirectionCommand(drivetrain, "operator"));
    }
}

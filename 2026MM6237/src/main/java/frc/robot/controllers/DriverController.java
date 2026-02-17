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

    public static double invertXNumberRobotCentric = -1.0;
    public static double invertYNumberRobotCentric = -1.0;

    public static Trigger robotCentricControl;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband).withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband).withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, NetworkTable limelight) {
        robotCentricControl = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold);

        Command defaultDrivetrainCommand = drivetrain.applyRequest(() -> {
            if (robotCentricControl.getAsBoolean()) {
                // Robot-centric control when left trigger is pressed
                return robotCentricDrive
                    .withVelocityX(invertXNumberRobotCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(invertYNumberRobotCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate); // Drive counterclockwise with negative X (left)
            } else {
                // Field-centric control (default)
                return drive
                    .withVelocityX(invertXNumberFieldCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(invertXNumberFieldCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate); // Drive counterclockwise with negative X (left)
            }
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

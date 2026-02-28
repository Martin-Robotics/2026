package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SnapToNearestAngleCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriverController {
    
    public static double invertXNumberFieldCentric = -1.0;
    public static double invertYNumberFieldCentric = -1.0;

    public static double invertXNumberRobotCentric = -1.0;
    public static double invertYNumberRobotCentric = -1.0;

    public static Trigger robotCentricControl;
    public static Trigger slowSpeedControl;
    public static Trigger fastSpeedControl;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband).withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband).withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, NetworkTable limelight) {
        robotCentricControl = new Trigger(() -> driverController.leftBumper().getAsBoolean());
        slowSpeedControl = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold);
        fastSpeedControl = new Trigger(() -> driverController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold);

        Command defaultDrivetrainCommand = drivetrain.applyRequest(() -> {
            // Determine speed multiplier based on triggers
            // Left trigger = 15%, Normal = 35%, Right trigger = 55%
            double speedMultiplier = 1.0; // Default is 35% (already in Constants.TempSwerve.MaxSpeed)
            if (slowSpeedControl.getAsBoolean()) {
                speedMultiplier = 15.0 / 35.0; // Scale to 15%
            } else if (fastSpeedControl.getAsBoolean()) {
                speedMultiplier = 55.0 / 35.0; // Scale to 55%
            }
            
            if (robotCentricControl.getAsBoolean()) {
                // Robot-centric control when left bumper is pressed
                return robotCentricDrive
                    .withVelocityX(invertXNumberRobotCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withVelocityY(invertYNumberRobotCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate * speedMultiplier);
            } else {
                // Field-centric control (default)
                return drive
                    .withVelocityX(invertXNumberFieldCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withVelocityY(invertYNumberFieldCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate * speedMultiplier);
            }
        });

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            defaultDrivetrainCommand
        );

        // A button: Snap to nearest 45-degree increment
        driverController.a().whileTrue(new SnapToNearestAngleCommand(drivetrain));
    }
}

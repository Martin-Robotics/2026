package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SnapToNearestAngleCommand;
import frc.robot.commands.auto.PrepareToFire;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;

public class DriverController {
    
    // Joystick axis conventions:
    // - getLeftY() is NEGATIVE when pushed FORWARD (away from driver)
    // - getLeftX() is POSITIVE when pushed RIGHT
    // 
    // FRC Field conventions (Blue Alliance perspective):
    // - Positive X = toward Red alliance wall (forward for Blue)
    // - Positive Y = toward left side of field (when standing at Blue driver station)
    //
    // With BlueAlliance perspective, Red drivers need controls inverted (handled below)
    public static double invertXNumberFieldCentric = -1.0;  // Negate because stick Y is inverted
    public static double invertYNumberFieldCentric = -1.0;  // Negate because we want left=positive

    public static double invertXNumberRobotCentric = -1.0;
    public static double invertYNumberRobotCentric = -1.0;

    public static Trigger robotCentricControl;
    public static Trigger slowSpeedControl;
    public static Trigger fastSpeedControl;

    // Field-centric drive with BlueAlliance perspective
    // We'll manually flip for Red alliance in the command
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);  // Standard FRC coordinates

    // Robot-centric drive - no perspective adjustment needed (always relative to robot)
    private static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, NetworkTable limelight, 
                                         Shooter shooter, LimelightSubsystem6237 limelightSubsystem) {
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
            
            // Check alliance - Red drivers need inverted field-centric controls
            boolean isRedAlliance = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red)
                .orElse(false);
            
            // Alliance multiplier: Red = -1 (flip), Blue = +1 (normal)
            double allianceMultiplier = isRedAlliance ? -1.0 : 1.0;
            
            // DEBUG: Alliance and control info
            SmartDashboard.putString("Drive/Alliance", isRedAlliance ? "RED" : "BLUE");
            SmartDashboard.putNumber("Drive/Alliance Multiplier", allianceMultiplier);
            SmartDashboard.putNumber("Drive/Raw Left Y", driverController.getLeftY());
            SmartDashboard.putNumber("Drive/Raw Left X", driverController.getLeftX());
            
            // Calculate velocities with alliance correction
            double velocityX = allianceMultiplier * invertXNumberFieldCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier;
            double velocityY = allianceMultiplier * invertYNumberFieldCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier;
            
            SmartDashboard.putNumber("Drive/VelocityX Cmd", velocityX);
            SmartDashboard.putNumber("Drive/VelocityY Cmd", velocityY);
            
            if (robotCentricControl.getAsBoolean()) {
                // Robot-centric control when left bumper is pressed (no alliance flip needed)
                return robotCentricDrive
                    .withVelocityX(invertXNumberRobotCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withVelocityY(invertYNumberRobotCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate * speedMultiplier);
            } else {
                // Field-centric control with alliance correction
                return drive
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate * speedMultiplier);
            }
        });

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            defaultDrivetrainCommand
        );

        // A button: Snap to nearest 45-degree increment
        driverController.a().whileTrue(new SnapToNearestAngleCommand(drivetrain));
        
        // Y button: PrepareToFire - Aim at hub and read distance
        driverController.y().whileTrue(new PrepareToFire(shooter, limelightSubsystem, drivetrain, driverController));
    }
}

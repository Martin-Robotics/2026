package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtHubWhileDriving;
import frc.robot.commands.SnapToNearestAngleCommand;
import frc.robot.commands.auto.PrepareToFire;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.HubMonitor;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriverController {
    
    // Joystick axis conventions:
    // - getLeftY() is NEGATIVE when pushed FORWARD (away from driver)
    // - getLeftX() is POSITIVE when pushed RIGHT
    // 
    // FRC Field conventions:
    // - With OperatorPerspective, "forward" always means "away from the driver station"
    //   regardless of alliance (CTRE handles the rotation via setOperatorPerspectiveForward)
    public static double invertXNumberFieldCentric = -1.0;  // Negate because stick Y is inverted
    public static double invertYNumberFieldCentric = -1.0;  // Negate because we want left=positive

    public static double invertXNumberRobotCentric = -1.0;
    public static double invertYNumberRobotCentric = -1.0;

    // Translation swap: when true, forward/back and left/right are reversed
    // Toggled by holding Start button for 1 second
    private static boolean translationSwapped = false;
    private static final Debouncer startDebouncer = new Debouncer(1.0, DebounceType.kRising);
    private static boolean lastDebouncedStart = false;

    /** Returns -1.0 when swapped (reverse translation), +1.0 normally */
    public static double getTranslationSwapMultiplier() {
        return translationSwapped ? -1.0 : 1.0;
    }

    public static Trigger robotCentricControl;
    public static Trigger slowSpeedControl;
    public static Trigger fastSpeedControl;

    // Field-centric drive with OperatorPerspective
    // CTRE's setOperatorPerspectiveForward() in CommandSwerveDrivetrain.periodic()
    // sets 0 deg for Blue, 180 deg for Red -- OperatorPerspective uses that value
    // so "forward" on the stick always means "away from the driver station"
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);  // Uses setOperatorPerspectiveForward per alliance

    // Robot-centric drive - no perspective adjustment needed (always relative to robot)
    private static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(Constants.TempSwerve.MaxSpeed * OperatorConstants.driverStickDeadband)
        .withRotationalDeadband(Constants.TempSwerve.MaxAngularRate * Constants.OperatorConstants.driverStickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, NetworkTable limelight, 
                                         Shooter shooter, LimelightSubsystem6237 limelightSubsystem, Hood hood,
                                         LEDSubsystem leds, HubMonitor hubMonitor) {
        robotCentricControl = new Trigger(() -> driverController.leftBumper().getAsBoolean());
        slowSpeedControl = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold);
        fastSpeedControl = new Trigger(() -> driverController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold);

        Command defaultDrivetrainCommand = drivetrain.applyRequest(() -> {
            // --- Translation swap: hold Start for 1 second to toggle ---
            boolean rawStart = driverController.start().getAsBoolean();
            boolean debouncedStart = startDebouncer.calculate(rawStart);
            // Detect rising edge of the debounced signal (just became true)
            if (debouncedStart && !lastDebouncedStart) {
                translationSwapped = !translationSwapped;
                System.out.println("[DriverController] Translation swap: " + (translationSwapped ? "REVERSED" : "NORMAL"));
            }
            lastDebouncedStart = debouncedStart;

            double swapMult = getTranslationSwapMultiplier();

            // Determine speed multiplier based on triggers
            // Left trigger = 15%, Normal = 35%, Right trigger = 55%
            double speedMultiplier = 1.0; // Default is 35% (already in Constants.TempSwerve.MaxSpeed)
            if (slowSpeedControl.getAsBoolean()) {
                speedMultiplier = 20.0 / 40.0; // Scale to 15%
            } else if (fastSpeedControl.getAsBoolean()) {
                speedMultiplier = 60.0 / 40.0; // Scale to 55%
            }
            
            // Calculate velocities -- swapMult flips both X and Y translation when toggled
            // CTRE OperatorPerspective handles alliance flip via setOperatorPerspectiveForward()
            double velocityX = swapMult * invertXNumberFieldCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier;
            double velocityY = swapMult * invertYNumberFieldCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier;
            
            if (robotCentricControl.getAsBoolean()) {
                // Robot-centric control when left bumper is pressed
                return robotCentricDrive
                    .withVelocityX(swapMult * invertXNumberRobotCentric * driverController.getLeftY() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withVelocityY(swapMult * invertYNumberRobotCentric * driverController.getLeftX() * Constants.TempSwerve.MaxSpeed * speedMultiplier)
                    .withRotationalRate(-1 * driverController.getRightX() * Constants.TempSwerve.MaxAngularRate * speedMultiplier);
            } else {
                // Field-centric control
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
        
        // Y button: PrepareToFire - Aim at hub, read distance, and position hood
        // Also activates targeting LED feedback while held
        driverController.y().whileTrue(new PrepareToFire(shooter, limelightSubsystem, drivetrain, driverController, hood));
        driverController.y()
            .onTrue(Commands.runOnce(() -> leds.setYTargetingActive(true)))
            .onFalse(Commands.runOnce(() -> leds.setYTargetingActive(false)));
        
        // B button: Toggle aim-at-hub mode - robot auto-rotates to face hub while driver retains full translation
        // Press B to activate, press B again to deactivate. Operator can fire while driver repositions.
        // Also toggles targeting LED state to match
        driverController.b().toggleOnTrue(new AimAtHubWhileDriving(limelightSubsystem, drivetrain, driverController));
        driverController.b()
            .onTrue(Commands.runOnce(() -> leds.toggleBTargeting()));

        // Hub activity indicator — LEDs show active/inactive hub pattern based on HubMonitor
        new Trigger(hubMonitor::isHubActive)
            .whileTrue(new RunCommand(() -> leds.setPattern(LEDSubsystem.Patterns.ACTIVE_HUB), leds))
            .whileFalse(new RunCommand(() -> leds.setPattern(LEDSubsystem.Patterns.INACTIVE_HUB), leds));
    }
}

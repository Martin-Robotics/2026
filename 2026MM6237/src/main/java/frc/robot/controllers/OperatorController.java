package frc.robot.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ShooterTuningCommand;
import frc.robot.commands.WCP.PrepareStaticShotCommand;
import frc.robot.commands.auto.PrepareToFire;
import frc.robot.commands.auto.PrepareToClimbLeft;
import frc.robot.commands.auto.PrepareToClimbRight;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Operator Controller mapping for testing and exercising subsystems.
 * 
 * Provides simple voltage control for all subsystems with intuitive button mappings:
 * - Left stick for continuous control
 * - Triggers for directional control
 * - Bumpers and face buttons for specific subsystems
 * 
 * Button Mapping:
 * - LB: Feeder forward (positive voltage)
 * - LT: Feeder reverse (negative voltage)
 * - RB: Shooter forward (positive voltage)
 * - RT: Shooter reverse (negative voltage)
 * - Left Stick Up: Intake pivot manual positive voltage (6% for testing)
 * - Left Stick Down: Intake pivot manual negative voltage (6% for testing)
 * - Left Stick Left: Seek STOWED position (while held)
 * - Left Stick Right: Seek INTAKE position (while held)
 * - Right Stick Forward: Hanger extend (minimal voltage for safe testing)
 * - Right Stick Backward: Hanger retract (minimal voltage for safe testing)
 * - Y: Hood servo UP (full up position - 1.0)
 * - X: Hood servo DOWN (full down position - 0.0)
 * - B: Intake agitate
 * - A: Intake homing
 * - DPad Up: Intake roller forward (positive voltage)
 * - DPad Down: Intake roller reverse (negative voltage)
 * - DPad Left: Floor feed forward
 * - DPad Right: Floor feed reverse
 * - Start: Prepare and shoot at static distance (shooter + hood + feeder + floor)
 */
public class OperatorController {
    
    // Control percentages for safe voltage testing
    private static final double MOTOR_SPEED_PERCENT = 0.3;  // 30% voltage for testing
    private static final double INTAKE_ROLLER_SPEED_PERCENT = 0.75; // 30% voltage for intake pivot testing
    private static final double INTAKE_SPEED_PERCENT = 0.06; // 6% voltage for intake testing (20% of 30%)
    private static final double HANGER_SPEED_PERCENT = 0.30; // 6% voltage for hanger testing (minimal for safety)
    
    // Shot preparation distance (meters)
    private static final double STATIC_SHOT_DISTANCE_METERS = 2.0; // Default test distance
    
    // Hood servo safe testing positions (centered around 0.5, small range for initial testing)
    private static final double HOOD_SAFE_UP_POSITION = 0.8;    // Slightly up from center
    private static final double HOOD_SAFE_DOWN_POSITION = 0.4;  // Slightly down from center
    private static final double LEFT_STICK_THRESHOLD = 0.15;     // Deadband for stick input
    private static final double RIGHT_STICK_THRESHOLD = 0.15;    // Deadband for stick input
    
    /**
     * Maps Xbox controller inputs to subsystem commands for testing and exercising subsystems.
     * This is the original test control scheme.
     * 
     * @param operatorController The Xbox controller for the operator
     * @param feeder The Feeder subsystem
     * @param shooter The Shooter subsystem
     * @param intake The Intake subsystem
     * @param hood The Hood subsystem
     * @param hanger The Hanger subsystem
     * @param floor The Floor subsystem
     */
    public static void mapXboxControllerTestInputs(
            CommandXboxController operatorController,
            Feeder feeder,
            Shooter shooter,
            Intake intake,
            Hood hood,
            Hanger hanger,
            Floor floor) {
        
        // ======================== FEEDER CONTROLS ========================
        // Left Bumper: Feeder forward
        operatorController.leftBumper()
            .whileTrue(feeder.runEnd(
                () -> feeder.setPercentOutput(MOTOR_SPEED_PERCENT),
                () -> feeder.setPercentOutput(0.0)
            ).withName("Feeder Forward"));
        
        // Left Trigger: Feeder reverse
        new Trigger(() -> operatorController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold)
            .whileTrue(feeder.runEnd(
                () -> feeder.setPercentOutput(-MOTOR_SPEED_PERCENT),
                () -> feeder.setPercentOutput(0.0)
            ).withName("Feeder Reverse"));
        
        // ======================== SHOOTER CONTROLS ========================
        // Right Bumper: Shooter forward
        operatorController.rightBumper()
            .whileTrue(shooter.runEnd(
                () -> shooter.setPercentOutput(MOTOR_SPEED_PERCENT),
                () -> shooter.setPercentOutput(0.0)
            ).withName("Shooter Forward"));
        
        // Right Trigger: Shooter reverse
        new Trigger(() -> operatorController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold)
            .whileTrue(shooter.runEnd(
                () -> shooter.setPercentOutput(-MOTOR_SPEED_PERCENT),
                () -> shooter.setPercentOutput(0.0)
            ).withName("Shooter Reverse"));
        
        // ======================== INTAKE CONTROLS ========================
        // Left Stick Up: Manual intake pivot positive voltage (for initial testing)
        // NOTE: Using reduced 6% voltage (20% of normal) for safe initial testing
        new Trigger(() -> operatorController.getLeftY() < -LEFT_STICK_THRESHOLD)
            .whileTrue(intake.runEnd(
                () -> intake.setManualPivotVoltage(INTAKE_SPEED_PERCENT),
                () -> intake.setManualPivotVoltage(0.0)
            ).withName("Intake Pivot Up (Manual)"));
        
        // Left Stick Down: Manual intake pivot negative voltage (for initial testing)
        new Trigger(() -> operatorController.getLeftY() > LEFT_STICK_THRESHOLD)
            .whileTrue(intake.runEnd(
                () -> intake.setManualPivotVoltage(-INTAKE_SPEED_PERCENT),
                () -> intake.setManualPivotVoltage(0.0)
            ).withName("Intake Pivot Down (Manual)"));
        
        // Left Stick Left: Seek STOWED position (only while held)
        // Command continuously sends position command while stick is held left
        // Uses manual position command for testing (bypasses homing check)
        new Trigger(() -> operatorController.getLeftX() < -LEFT_STICK_THRESHOLD)
            .whileTrue(intake.run(() -> intake.setManualPosition(Intake.Position.STOWED))
                .withName("Seek Stowed Position"));
        
        // Left Stick Right: Seek INTAKE position (only while held)
        // Command continuously sends position command while stick is held right
        // Uses manual position command for testing (bypasses homing check)
        new Trigger(() -> operatorController.getLeftX() > LEFT_STICK_THRESHOLD)
            .whileTrue(intake.run(() -> intake.setManualPosition(Intake.Position.INTAKE))
                .withName("Seek Intake Position"));
        
        // B Button: Intake agitate command
        operatorController.b()
            .onTrue(intake.agitateCommand().withName("Intake Agitate"));
        
        // A Button: Intake homing command  
        operatorController.a()
            .onTrue(intake.homingCommand().withName("Intake Homing"));
        
        // ======================== HOOD CONTROLS ========================
        // Y Button: Hood servo UP (safe testing - small movement to 0.6)
        // SAFETY NOTE: These positions are close to center (0.5) for initial testing
        // since the hood is not confirmed to be wired/working. This prevents large
        // movements that could cause mechanical damage during initial testing.
        operatorController.y()
            .onTrue(hood.runOnce(() -> hood.setPosition(HOOD_SAFE_UP_POSITION))
                .withName("Hood Up (Safe Test)"));
        
        // X Button: Hood servo DOWN (safe testing - small movement to 0.4)
        operatorController.x()
            .onTrue(hood.runOnce(() -> hood.setPosition(HOOD_SAFE_DOWN_POSITION))
                .withName("Hood Down (Safe Test)"));

        // ======================== HANGER CONTROLS ========================
        // Right Stick Forward: Hanger extend (minimal voltage for safe testing)
        // SAFETY NOTE: Using 6% voltage for initial testing to prevent damage
        // to the climb mechanism. This allows safe verification of motor direction
        // and mechanical operation before using full power.
        new Trigger(() -> operatorController.getRightY() < -RIGHT_STICK_THRESHOLD)
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(HANGER_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Extend (Safe Test)"));
        
        // Right Stick Backward: Hanger retract (minimal voltage for safe testing)
        new Trigger(() -> operatorController.getRightY() > RIGHT_STICK_THRESHOLD)
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(-HANGER_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Retract (Safe Test)"));

        // ======================== INTAKE ROLLER CONTROLS ========================
        // DPad Up: Intake roller forward (positive voltage)
        operatorController.pov(0)
            .whileTrue(intake.runEnd(
                () -> intake.setManualRollerVoltage(INTAKE_ROLLER_SPEED_PERCENT),
                () -> intake.setManualRollerVoltage(0.0)
            ).withName("Intake Roller Forward"));
        
        // DPad Down: Intake roller reverse (negative voltage)  
        operatorController.pov(180)
            .whileTrue(intake.runEnd(
                () -> intake.setManualRollerVoltage(-INTAKE_ROLLER_SPEED_PERCENT),
                () -> intake.setManualRollerVoltage(0.0)
            ).withName("Intake Roller Reverse"));
        
        // ======================== FLOOR CONTROLS ========================
        // DPad Left: Floor feed forward
        operatorController.pov(270)
            .whileTrue(floor.runEnd(
                () -> floor.set(Floor.Speed.FEED),
                () -> floor.set(Floor.Speed.STOP)
            ).withName("Floor Feed Forward"));
        
        // DPad Right: Floor feed reverse
        operatorController.pov(90)
            .whileTrue(floor.runEnd(
                () -> floor.set(Floor.Speed.REVERSE),
                () -> floor.set(Floor.Speed.STOP)
            ).withName("Floor Feed Reverse"));
        
        // ======================== SHOT PREPARATION ========================
        // Start Button: Prepare shot at static distance (for testing without odometry)
        // Spins up shooter, positions hood, and engages feeder and floor when at speed
        // operatorController.start()
        //     .whileTrue(new PrepareStaticShotCommand(shooter, hood, feeder, floor, STATIC_SHOT_DISTANCE_METERS) STATIC_SHOT_DISTANCE_METERS = 2.0
        //         .withName("Prepare Static Shot"));
        operatorController.start()
            .whileTrue(new PrepareStaticShotCommand(shooter, hood, feeder, floor, 3) 
                .withName("Prepare Static Shot"));
    }

    /**
     * Maps Xbox controller inputs to subsystem commands for competition control scheme.
     * 
     * Button Mapping:
     * - RT (Right Trigger): PrepareStaticShotCommand (prepares and shoots at auto-detected distance)
     * - RB (Right Bumper): Hanger extend
     * - LB (Left Bumper): Hanger retract
     * - A: Move intake arm to INTAKE position (independent of rollers)
     * - B: Move intake arm to STOWED position (also stops rollers)
     * - X: Run intake rollers (blocked if arm is in STOWED position)
     * - Y: Stop intake rollers
     * - Back: PrepareToClimbLeft
     * - Start: PrepareToClimbRight
     * 
     * Note: PrepareToFire (aim and distance detection) is on Driver Y button
     * 
     * @param operatorController The Xbox controller for the operator
     * @param feeder The Feeder subsystem
     * @param shooter The Shooter subsystem
     * @param intake The Intake subsystem
     * @param hood The Hood subsystem
     * @param hanger The Hanger subsystem
     * @param floor The Floor subsystem
     * @param limelight The Limelight subsystem
     * @param drivetrain The CommandSwerveDrivetrain (not used here, but kept for compatibility)
     * @param driverController The driver's Xbox controller (not used here, but kept for compatibility)
     */
    public static void mapXboxController(
            CommandXboxController operatorController,
            Feeder feeder,
            Shooter shooter,
            Intake intake,
            Hood hood,
            Hanger hanger,
            Floor floor,
            LimelightSubsystem6237 limelight,
            frc.robot.subsystems.CommandSwerveDrivetrain drivetrain,
            CommandXboxController driverController) {
        
        // ======================== SHOOTING CONTROLS ========================
        // Right Trigger: PrepareStaticShotCommand (uses background-tracked Limelight distance)
        // Distance is continuously tracked by LimelightSubsystem in the background
        // If no distance detected, falls back to 3 meters
        new Trigger(() -> operatorController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold)
            .whileTrue(new PrepareStaticShotCommand(shooter, hood, feeder, floor, 3.0, true, limelight)
                .withName("Prepare Static Shot (Auto Distance)"));
        
        // NOTE: PrepareToFire (aiming) is now mapped to Driver Y button (see DriverController)
        
        // ======================== HANGER CONTROLS ========================
        // Right Bumper: Hanger extend
        operatorController.rightBumper()
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(HANGER_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Extend"));
        
        // Left Bumper: Hanger retract
        operatorController.leftBumper()
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(-HANGER_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Retract"));
        
        // ======================== INTAKE CONTROLS ========================
        // A Button: Move intake arm to INTAKE position (does not run rollers)
        operatorController.a()
            .onTrue(intake.runOnce(() -> {
                intake.setManualPosition(Intake.Position.INTAKE);
            }).withName("Extend Intake Arm"));
        
        // B Button: Move intake arm to STOWED position (also stops rollers)
        operatorController.b()
            .onTrue(intake.runOnce(() -> {
                intake.setManualPosition(Intake.Position.STOWED);
                intake.set(Intake.Speed.STOP);
            }).withName("Retract Intake Arm"));
        
        // X Button: Run intake rollers
        operatorController.x()
            .onTrue(intake.runOnce(() -> {
                intake.set(Intake.Speed.INTAKE);
            }).withName("Start Intake Rollers"));
        
        // Y Button: Stop intake rollers
        operatorController.y()
            .onTrue(intake.runOnce(() -> {
                intake.set(Intake.Speed.STOP);
            }).withName("Stop Intake Rollers"));
        
        // ======================== CLIMB PREPARATION ========================
        // Back Button: PrepareToClimbLeft
        operatorController.back()
            .onTrue(new PrepareToClimbLeft(hanger)
                .withName("Prepare To Climb Left"));
        
        // Start Button: PrepareToClimbRight
        operatorController.start()
            .onTrue(new PrepareToClimbRight(hanger)
                .withName("Prepare To Climb Right"));
        
        // ======================== SHOOTER TUNING MODE ========================
        // DPad Up (POV 0): Hold for Shooter Tuning Mode
        // Adjust RPM and Hood via SmartDashboard while this is held
        // Use RT (Right Trigger) to fire test shots while in tuning mode
        // This command requires shooter, hood, feeder, AND floor so RT won't trigger PrepareStaticShot
        new Trigger(() -> operatorController.getHID().getPOV() == 0)
            .whileTrue(new ShooterTuningCommand(shooter, hood, feeder, floor, limelight, operatorController)
                .withName("Shooter Tuning Mode"));
        
        // ======================== MANUAL HOOD CONTROLS (FOR TESTING) ========================
        // DPad Left (POV 270): Hood DOWN (lower position number = flatter trajectory)
        // Hold to move hood toward 0.1
        new Trigger(() -> operatorController.getHID().getPOV() == 270)
            .whileTrue(hood.run(() -> {
                double current = hood.getTargetPosition();
                double newPos = Math.max(0.1, current - 0.02); // Step down by 0.02 each cycle
                hood.setPosition(newPos);
                SmartDashboard.putNumber("Hood/Manual Target", newPos);
            }).withName("Hood DOWN (Manual)"));
        
        // DPad Right (POV 90): Hood UP (higher position number = steeper arc)
        // Hold to move hood toward 0.75
        new Trigger(() -> operatorController.getHID().getPOV() == 90)
            .whileTrue(hood.run(() -> {
                double current = hood.getTargetPosition();
                double newPos = Math.min(0.75, current + 0.02); // Step up by 0.02 each cycle
                hood.setPosition(newPos);
                SmartDashboard.putNumber("Hood/Manual Target", newPos);
            }).withName("Hood UP (Manual)"));
    }
}

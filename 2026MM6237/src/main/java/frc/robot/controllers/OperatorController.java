package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
 * - Y: Hood servo UP (full up position - 1.0)
 * - X: Hood servo DOWN (full down position - 0.0)
 * - B: Intake agitate
 * - A: Intake homing
 * - DPad Up: Hanger extend (positive voltage)
 * - DPad Down: Hanger retract (negative voltage)
 * - DPad Left: Floor feed forward
 * - DPad Right: Floor feed reverse
 */
public class OperatorController {
    
    // Control percentages for safe voltage testing
    private static final double MOTOR_SPEED_PERCENT = 0.3;  // 30% voltage for testing
    private static final double INTAKE_SPEED_PERCENT = 0.06; // 6% voltage for intake testing (20% of 30%)
    
    // Hood servo safe testing positions (centered around 0.5, small range for initial testing)
    private static final double HOOD_SAFE_UP_POSITION = 0.8;    // Slightly up from center
    private static final double HOOD_SAFE_DOWN_POSITION = 0.4;  // Slightly down from center
    private static final double LEFT_STICK_THRESHOLD = 0.15;     // Deadband for stick input
    
    /**
     * Maps Xbox controller inputs to subsystem commands.
     * 
     * @param operatorController The Xbox controller for the operator
     * @param feeder The Feeder subsystem
     * @param shooter The Shooter subsystem
     * @param intake The Intake subsystem
     * @param hood The Hood subsystem
     * @param hanger The Hanger subsystem
     * @param floor The Floor subsystem
     */
    public static void mapXboxController(
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
        // DPad Up: Hanger extend (positive voltage)
        operatorController.pov(0)
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(MOTOR_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Extend"));
        
        // DPad Down: Hanger retract (negative voltage)
        operatorController.pov(180)
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(-MOTOR_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Retract"));
        
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
    }
}

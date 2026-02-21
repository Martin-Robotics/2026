package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED(0),
        EXTEND_HOPPER(Constants.Hanger.kExtendHopperInches),
        HANGING(Constants.Hanger.kHangingInches),
        HUNG(Constants.Hanger.kHungInches);

        private final double inches;

        private Position(double inches) {
            this.inches = inches;
        }

        public Angle motorAngle() {
            final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
            return Rotations.of(angleMeasure.in(Rotations)); // Promote from Measure<AngleUnit> to Angle
        }
    }

    private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(Constants.Hanger.kHangerExtensionInches).div(Rotations.of(Constants.Hanger.kHangerExtensionMotorRotations));
    private static final Distance kExtensionTolerance = Constants.Hanger.kExtensionTolerance;

    private final SparkMax masterMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    
    private Angle targetPosition = Rotations.zero();

    private boolean isHomed = false;

    public Hanger() {
        // Master motor - controls the hanger position
        masterMotor = new SparkMax(Ports.kHanger, MotorType.kBrushless);
        encoder = masterMotor.getEncoder();
        pidController = masterMotor.getClosedLoopController();
        
        // Follower motor - runs in sync with master
        followerMotor = new SparkMax(Ports.kHangerFollower, MotorType.kBrushless);

        final SparkMaxConfig config = new SparkMaxConfig();
        
        // Motor output configuration
        config.inverted(true); // Equivalent to Clockwise_Positive
        config.idleMode(IdleMode.kBrake);
        
        // Current limits
        config.smartCurrentLimit((int) Constants.Hanger.kStatorCurrentLimit);
        
        // PID configuration for slot 0
        config.closedLoop.p(Constants.Hanger.kP);
        config.closedLoop.i(Constants.Hanger.kI);
        config.closedLoop.d(Constants.Hanger.kD);
        config.closedLoop.outputRange(-1, 1);

        // Apply configuration to master motor
        masterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        // Configure follower motor with same settings but potentially different inversion
        final SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.inverted(true); // Change to false if follower needs opposite direction
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.smartCurrentLimit((int) Constants.Hanger.kStatorCurrentLimit);
        followerConfig.follow(Ports.kHanger); // Follow the master motor
        
        followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        // SAFETY: Ensure motor starts with zero voltage output
        neutralizeMotor();
        
        SmartDashboard.putData(this);
    }

    /**
     * Ensures the hanger motor starts with zero voltage output.
     * Called during initialization to prevent unintended motor motion on enable.
     */
    private void neutralizeMotor() {
        masterMotor.setVoltage(0);
    }

    public void set(Position position) {
        targetPosition = position.motorAngle();
        pidController.setReference(targetPosition.in(Rotations), ControlType.kPosition);
    }

    public void setPercentOutput(double percentOutput) {
        masterMotor.set(percentOutput);
    }

    public Command positionCommand(Position position) {
        return runOnce(() -> set(position))
            .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(Constants.Hanger.kHomingPercentOutput)),
            Commands.waitUntil(() -> masterMotor.getOutputCurrent() > Constants.Hanger.kHomingCurrentThreshold),
            runOnce(() -> {
                encoder.setPosition(Position.HOMED.motorAngle().in(Rotations));
                isHomed = true;
                set(Position.EXTEND_HOPPER);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean isHomed() {
        return isHomed;
    }

    // ======================== GETTER METHODS FOR TUNING ========================

    public double getCurrentExtensionInches() {
        return motorAngleToExtension(Rotations.of(encoder.getPosition())).in(Inches);
    }

    public double getSupplyCurrent() {
        return masterMotor.getOutputCurrent();
    }

    private boolean isExtensionWithinTolerance() {
        final Distance currentExtension = motorAngleToExtension(Rotations.of(encoder.getPosition()));
        final Distance targetExtension = motorAngleToExtension(targetPosition);
        return currentExtension.isNear(targetExtension, kExtensionTolerance);
    }

    private Distance motorAngleToExtension(Angle motorAngle) {
        final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure.in(Inches)); // Promote from Measure<DistanceUnit> to Distance
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Extension (inches)", () -> motorAngleToExtension(Rotations.of(encoder.getPosition())).in(Inches), null);
        builder.addDoubleProperty("Supply Current", () -> masterMotor.getOutputCurrent(), null);
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

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

    private final SparkMax motor;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    
    private double targetPositionRotations = 0;

    private boolean isHomed = false;

    public Hanger() {
        motor = new SparkMax(Ports.kHanger, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        // Create configuration
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true) // Equivalent to Clockwise_Positive
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int)Constants.Hanger.kStatorCurrentLimit)
            .secondaryCurrentLimit(Constants.Hanger.kSupplyCurrentLimit);
        
        // Configure PID and Smart Motion
        config.closedLoop
            .pidf(Constants.Hanger.kP, Constants.Hanger.kI, Constants.Hanger.kD, 12.0 / Constants.NEO.kFreeSpeed.in(RotationsPerSecond));
        
        config.closedLoop.maxMotion
            .maxVelocity(Constants.NEO.kFreeSpeed.in(RotationsPerSecond))
            .maxAcceleration(Constants.NEO.kFreeSpeed.in(RotationsPerSecond))
            .allowedClosedLoopError(0.1);
        
        // Apply configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // SAFETY: Ensure motor starts with zero voltage output
        neutralizeMotor();
        
        // SmartDashboard.putData(this); // Commented out to reduce dashboard clutter
    }

    /**
     * Ensures the hanger motor starts with zero voltage output.
     * Called during initialization to prevent unintended motor motion on enable.
     */
    private void neutralizeMotor() {
        motor.setVoltage(0);
    }

    public void set(Position position) {
        targetPositionRotations = position.motorAngle().in(Rotations);
        closedLoopController.setReference(
            targetPositionRotations,
            SparkMax.ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    public Command positionCommand(Position position) {
        return runOnce(() -> set(position))
            .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(Constants.Hanger.kHomingPercentOutput)),
            Commands.waitUntil(() -> motor.getOutputCurrent() > Constants.Hanger.kHomingCurrentThreshold),
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
        return motor.getOutputCurrent();
    }

    private boolean isExtensionWithinTolerance() {
        final Distance currentExtension = motorAngleToExtension(Rotations.of(encoder.getPosition()));
        final Distance targetExtension = motorAngleToExtension(Rotations.of(targetPositionRotations));
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
        builder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
    }
}

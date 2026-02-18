package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.Constants.KrakenX60;
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

    private final TalonFX motor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private boolean isHomed = false;

    public Hanger() {
        motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.Hanger.kStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(Constants.Hanger.kSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(Constants.Hanger.kP)
                    .withKI(Constants.Hanger.kI)
                    .withKD(Constants.Hanger.kD)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );

        motor.getConfigurator().apply(config);
        
        // SAFETY: Ensure motor starts with zero voltage output
        neutralizeMotor();
        
        SmartDashboard.putData(this);
    }

    /**
     * Ensures the hanger motor starts with zero voltage output.
     * Called during initialization to prevent unintended motor motion on enable.
     */
    private void neutralizeMotor() {
        motor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    public void set(Position position) {
        motor.setControl(
            motionMagicRequest
                .withPosition(position.motorAngle())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public Command positionCommand(Position position) {
        return runOnce(() -> set(position))
            .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(Constants.Hanger.kHomingPercentOutput)),
            Commands.waitUntil(() -> motor.getSupplyCurrent().getValue().in(Amps) > Constants.Hanger.kHomingCurrentThreshold),
            runOnce(() -> {
                motor.setPosition(Position.HOMED.motorAngle());
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
        return motorAngleToExtension(motor.getPosition().getValue()).in(Inches);
    }

    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValue().in(Amps);
    }

    private boolean isExtensionWithinTolerance() {
        final Distance currentExtension = motorAngleToExtension(motor.getPosition().getValue());
        final Distance targetExtension = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
        return currentExtension.isNear(targetExtension, kExtensionTolerance);
    }

    private Distance motorAngleToExtension(Angle motorAngle) {
        final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure.in(Inches)); // Promote from Measure<DistanceUnit> to Distance
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Extension (inches)", () -> motorAngleToExtension(motor.getPosition().getValue()).in(Inches), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}

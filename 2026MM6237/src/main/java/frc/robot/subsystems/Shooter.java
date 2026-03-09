package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = Constants.Shooter.kVelocityTolerance;

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage middleVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;
    private double leftTargetRPM = 0.0;
    private double middleTargetRPM = 0.0;
    private double rightTargetRPM = 0.0;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive, Constants.Shooter.kLeftKP);
        // configureMotor(middleMotor, InvertedValue.Clockwise_Positive);'
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive, Constants.Shooter.kMiddleKP);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive, Constants.Shooter.kRightKP);

        // SAFETY: Ensure motors start with zero voltage output
        neutralizeMotors();
        
        // SmartDashboard.putData(this); // Commented out to reduce dashboard clutter
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection, double kP) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(Constants.Shooter.kPeakReverseVoltage))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.Shooter.kStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(Constants.Shooter.kSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(kP)
                    .withKI(Constants.Shooter.kKI)
                    .withKD(Constants.Shooter.kKD)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    /**
     * Ensures all shooter motors start with zero voltage output.
     * Called during initialization to prevent unintended motor spin on enable.
     */
    private void neutralizeMotors() {
        for (final TalonFX motor : motors) {
            motor.setControl(voltageRequest.withOutput(Volts.of(0)));
        }
    }

    public void setRPM(double rpm) {
        // Left and middle motors use the base RPM
        leftTargetRPM = rpm;
        leftMotor.setControl(
            leftVelocityRequest
                .withVelocity(RPM.of(leftTargetRPM))
        );
        
        middleTargetRPM = rpm;
        middleMotor.setControl(
            middleVelocityRequest
                .withVelocity(RPM.of(middleTargetRPM))
        );
        
        // Apply 25% increase to right motor to compensate for decreased wheel width
        rightTargetRPM = rpm * 1.25;
        rightMotor.setControl(
            rightVelocityRequest
                .withVelocity(RPM.of(rightTargetRPM))
        );
        
        // Debug output to verify commands (commented out after testing)
        // SmartDashboard.putNumber("Shooter/Commanded Left RPM", leftTargetRPM);
        // SmartDashboard.putNumber("Shooter/Commanded Middle RPM", middleTargetRPM);
        // SmartDashboard.putNumber("Shooter/Commanded Right RPM", rightTargetRPM);
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public boolean isVelocityWithinTolerance() {
        final boolean leftAtSpeed = leftMotor.getVelocity().getValue()
            .isNear(RPM.of(leftTargetRPM), kVelocityTolerance);
        final boolean middleAtSpeed = middleMotor.getVelocity().getValue()
            .isNear(RPM.of(middleTargetRPM), kVelocityTolerance);
        final boolean rightAtSpeed = rightMotor.getVelocity().getValue()
            .isNear(RPM.of(rightTargetRPM), kVelocityTolerance);
        
        return leftAtSpeed && middleAtSpeed && rightAtSpeed;
    }

    // ======================== GETTER METHODS FOR TUNING ========================

    public double getLeftMotorRPM() {
        return leftMotor.getVelocity().getValue().in(RPM);
    }

    public double getMiddleMotorRPM() {
        return middleMotor.getVelocity().getValue().in(RPM);
    }

    public double getRightMotorRPM() {
        return rightMotor.getVelocity().getValue().in(RPM);
    }

    public double getLeftStatorCurrent() {
        return leftMotor.getStatorCurrent().getValue().in(Amps);
    }

    public double getMiddleStatorCurrent() {
        return middleMotor.getStatorCurrent().getValue().in(Amps);
    }

    public double getRightStatorCurrent() {
        return rightMotor.getStatorCurrent().getValue().in(Amps);
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Left Target RPM", () -> leftTargetRPM, null);
        builder.addDoubleProperty("Middle Target RPM", () -> middleTargetRPM, null);
        builder.addDoubleProperty("Right Target RPM", () -> rightTargetRPM, null);
    }
}

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(Constants.Intake.kIntakePercentOutput);
        private final double percentOutput;
        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }
        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }
    public enum Position {
        HOMED(Constants.Intake.kHomedPositionDegrees),
        STOWED(Constants.Intake.kStowedPositionDegrees),
        INTAKE(Constants.Intake.kIntakePositionDegrees),
        AGITATE(Constants.Intake.kAgitatePositionDegrees);
        private final double degrees;
        private Position(double degrees) {
            this.degrees = degrees;
        }
        public Angle angle() {
            return Degrees.of(degrees);
        }
    }
    private static final double kPivotReduction = Constants.Intake.kPivotReduction;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Constants.Intake.kPositionTolerance;
    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private boolean isHomed = false;
    public Intake() {
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kRoboRioCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
        configurePivotMotor();
        configureRollerMotor();
        
        // SAFETY: Ensure motors start in neutral state with zero voltage
        // This prevents any unintended motion on enable before calibration
        neutralizeMotors();
        
        // SmartDashboard.putData(this); // Commented out to reduce dashboard clutter
    }
    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.Intake.kStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(Constants.Intake.kSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    // Set to 40% of max speed for faster position seeking
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed.times(0.3))
                    .withMotionMagicAcceleration(kMaxPivotSpeed.times(0.3).per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(Constants.Intake.kPivotKP)
                    .withKI(Constants.Intake.kPivotKI)
                    .withKD(Constants.Intake.kPivotKD)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);
    }
    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.Intake.kStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(Constants.Intake.kSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }
    /**
     * Ensures both motors start with zero voltage output.
     * Called during initialization to prevent unintended motion on enable.
     */
    private void neutralizeMotors() {
        // Set pivot motor to zero voltage (no position command)
        pivotMotor.setControl(pivotVoltageRequest.withOutput(Volts.of(0)));
        // Set roller motor to zero voltage
        rollerMotor.setControl(rollerVoltageRequest.withOutput(Volts.of(0)));
    }
    private boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }
    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }
    
    /**
     * Manually sets the pivot motor voltage for testing purposes.
     * Use this for initial testing before homing to verify motor direction.
     * 
     * @param percentOutput Percent output from -1.0 to 1.0
     */
    public void setManualPivotVoltage(double percentOutput) {
        setPivotPercentOutput(percentOutput);
    }
    
    /**
     * Manually sets the roller motor voltage for testing purposes.
     * Use this to verify motor direction and test roller operation.
     * 
     * @param percentOutput Percent output from -1.0 to 1.0
     */
    public void setManualRollerVoltage(double percentOutput) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }
    
    /**
     * Manually sets the pivot position for testing purposes, bypassing homing check.
     * Use this ONLY for testing after you've manually established a zero point.
     * For normal operation, use set(Position) after homing.
     * 
     * @param position The target position for the intake pivot
     */
    public void setManualPosition(Position position) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
    }
    
    /**
     * Commands the pivot to move to a specified position.
     * SAFETY: This method will ignore commands if the intake has not been homed.
     * 
     * @param position The target position for the intake pivot
     */
    public void set(Position position) {
        if (!isHomed) {
            // Safety: Don't send position commands until homing is complete
            System.err.println("WARNING: Intake position command ignored - not homed!");
            return;
        }
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
    }
    public void set(Speed speed) {
        // Safety: Don't run rollers if pivot is in or seeking stowed position
        if (speed != Speed.STOP) {
            double targetAngle = pivotMotionMagicRequest.getPositionMeasure().in(Degrees);
            double stowedAngle = Position.STOWED.angle().in(Degrees);
            
            // Check if target position is stowed (within tolerance)
            if (Math.abs(targetAngle - stowedAngle) < kPositionTolerance.in(Degrees)) {
                System.err.println("WARNING: Rollers cannot run while intake is in STOWED position!");
                return;
            }
        }
        
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }
    public Command intakeCommand() {
        return Commands.sequence(
            runOnce(() -> {
                set(Position.INTAKE);
                set(Speed.INTAKE);
            }),
            Commands.waitUntil(this::isPositionWithinTolerance)
        ).finallyDo((interrupted) -> set(Speed.STOP));
    }
    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .finallyDo((interrupted) -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }
    /**
     * Homes the intake pivot by driving into a hard stop until current spike is detected.
     * After homing, the intake automatically moves to the STOWED position and waits
     * for the motion to complete before finishing.
     * 
     * This command will not run if the intake is already homed, and cannot be interrupted
     * once started for safety reasons.
     * 
     * @return Command that homes the intake and moves to stowed position
     */
    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(Constants.Intake.kHomingPercentOutput)),
            Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > Constants.Intake.kHomingCurrentThreshold),
            runOnce(() -> {
                pivotMotor.setPosition(Position.HOMED.angle());
                isHomed = true;
                set(Position.STOWED);
            }),
            Commands.waitUntil(this::isPositionWithinTolerance)  // Wait for stowed position
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    // ======================== GETTER METHODS FOR TUNING ========================
    public double getPivotAngleDegrees() {
        return pivotMotor.getPosition().getValue().in(Degrees);
    }
    public double getRollerRPM() {
        return rollerMotor.getVelocity().getValue().in(RPM);
    }
    public double getPivotStatorCurrent() {
        return pivotMotor.getStatorCurrent().getValue().in(Amps);
    }
    public double getRollerStatorCurrent() {
        return rollerMotor.getStatorCurrent().getValue().in(Amps);
    }
    public boolean isHomed() {
        return isHomed;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addBooleanProperty("Is Homed", this::isHomed, null);
    }
}

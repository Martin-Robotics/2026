// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kTriggerButtonThreshold = 0.3;
    public static final double driverStickDeadband = 0.2;
    public static final double operatorStickDeadband = 0.2;
  }

  public static class TempSwerve {
    public static final double MaxSpeed = 2.0; // meters per second
    public static final double MaxAngularRate = 2 * Math.PI; // radians per second
  }
  
    public static class Driving {
      public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
      public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
      public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class Feeder {
        public static final double kFeedRPM = 5000;
        public static final double kStatorCurrentLimit = 120;
        public static final double kSupplyCurrentLimit = 50;
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kVoltageAtMaxSpeed = 12.0;

        // ======================== AUTONOMOUS CONSTANTS ========================
        // Fire command duration (seconds)
        public static final double kAutoFireDurationSeconds = 0.5;

        // Default feeder output when distance cannot be determined from Limelight
        public static final double kAutoDefaultFeederPercentOutput = 0.8;

        // Distance-based feeder speed calculation parameters
        public static final double kAutoMinFeederDistanceMeters = 1.0;
        public static final double kAutoMaxFeederDistanceMeters = 8.0;
        public static final double kAutoMinFeederPercentOutput = 0.6;
        public static final double kAutoMaxFeederPercentOutput = 1.0;
    }

    public static class Floor {
        public static final double kFeedPercentOutput = 0.83;
        public static final double kStatorCurrentLimit = 120;
        public static final double kSupplyCurrentLimit = 30;
        public static final double kVoltageMultiplier = 12.0;
    }

    public static class Hanger {
        public static final double kExtendHopperInches = 2;
        public static final double kHangingInches = 6;
        public static final double kHungInches = 0.2;
        public static final double kHangerExtensionInches = 6;
        public static final double kHangerExtensionMotorRotations = 142;
        public static final Distance kExtensionTolerance = Inches.of(1);
        public static final double kStatorCurrentLimit = 20;
        public static final double kSupplyCurrentLimit = 70;
        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kHomingPercentOutput = -0.05;
        public static final double kHomingCurrentThreshold = 0.4;

        // ======================== AUTONOMOUS CONSTANTS ========================
        // Extension tolerance for hanger positioning in autonomous (inches)
        public static final double kAutoExtensionToleranceInches = 1.0;

        // Conversion factor from motor rotations to inches
        // (approximate multiplier for hanger extension calculation)
        public static final double kAutoRotationsToInchesMultiplier = 2.0;
    }

    public static class Hood {
        public static final Distance kServoLength = Millimeters.of(100);
        public static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
        public static final double kMinPosition = 0.01;
        public static final double kMaxPosition = 0.77;
        public static final double kPositionTolerance = 0.01;
        public static final int kServoBoundMax = 2000;
        public static final int kServoBoundHigh = 1800;
        public static final int kServoBoundCenter = 1500;
        public static final int kServoBoundLow = 1200;
        public static final int kServoBoundMin = 1000;
        public static final double kInitialPosition = 0.5;
    }

    public static class Intake {
        public static final double kIntakePercentOutput = 0.8;
        public static final double kHomedPositionDegrees = 110;
        public static final double kStowedPositionDegrees = 100;
        public static final double kIntakePositionDegrees = -4;
        public static final double kAgitatePositionDegrees = 20;
        public static final double kPivotReduction = 50.0;
        public static final Angle kPositionTolerance = Degrees.of(5);
        public static final double kStatorCurrentLimit = 120;
        public static final double kSupplyCurrentLimit = 70;
        public static final double kPivotKP = 300;
        public static final double kPivotKI = 0;
        public static final double kPivotKD = 0;
        public static final double kHomingPercentOutput = 0.1;
        public static final double kHomingCurrentThreshold = 6;

        // ======================== AUTONOMOUS CONSTANTS ========================
        // Position tolerance for intake arm motion in autonomous (degrees)
        public static final double kAutoPositionToleranceDegrees = 5.0;
    }

    public static class Shooter {
        public static final AngularVelocity kVelocityTolerance = RPM.of(100);
        public static final double kStatorCurrentLimit = 120;
        public static final double kSupplyCurrentLimit = 70;
        public static final double kLeftKP = 0.5;
        public static final double kMiddleKP = 0.5;
        public static final double kRightKP = 0.5;
        public static final double kKI = 2;
        public static final double kKD = 0;
        public static final double kPeakReverseVoltage = 0;

        // ======================== AUTONOMOUS CONSTANTS ========================
        // Default shooter RPM when distance cannot be determined from Limelight
        public static final double kAutoDefaultShooterRPM = 5000;

        // Distance-based shooter RPM calculation parameters
        public static final double kAutoMinShootingDistanceMeters = 1.0;
        public static final double kAutoMaxShootingDistanceMeters = 8.0;
        public static final double kAutoMinShooterRPM = 2000;
        public static final double kAutoMaxShooterRPM = 5500;
    }

    public static class Limelight {
        public static final double kStandardDeviationX = 0.1;
        public static final double kStandardDeviationY = 0.1;
        public static final double kStandardDeviationTheta = 10.0;
    }

    // public static class CommandSwerveDrivetrainOld {
    //     public static final double kSimLoopPeriod = 0.005; // 5 ms
    //     public static final double kTranslationPIDKP = 10;
    //     public static final double kTranslationPIDKI = 0;
    //     public static final double kTranslationPIDKD = 0;
    //     public static final double kRotationPIDKP = 7;
    //     public static final double kRotationPIDKI = 0;
    //     public static final double kRotationPIDKD = 0;
    //     public static final double kTranslationCharacterizationDynamicVoltage = 4;
    //     public static final double kSteerCharacterizationDynamicVoltage = 7;
    //     public static final double kRotationCharacterizationRampRateVoltage = Math.PI / 6;
    //     public static final double kRotationCharacterizationDynamicVoltage = Math.PI;

    //     // ======================== STEER MOTOR GAINS ========================
    //     public static final double kSteerMotorKP = 100;
    //     public static final double kSteerMotorKI = 0;
    //     public static final double kSteerMotorKD = 0.5;
    //     public static final double kSteerMotorKS = 0.1;  // Static feedforward
    //     public static final double kSteerMotorKV = 2.66; // Velocity feedforward
    //     public static final double kSteerMotorKA = 0;    // Acceleration feedforward

    //     // ======================== DRIVE MOTOR GAINS ========================
    //     public static final double kDriveMotorKP = 0.1;
    //     public static final double kDriveMotorKI = 0;
    //     public static final double kDriveMotorKD = 0;
    //     public static final double kDriveMotorKS = 0;      // Static feedforward
    //     public static final double kDriveMotorKV = 0.124;  // Velocity feedforward

    //     // ======================== MOTOR CURRENT LIMITS ========================
    //     public static final double kSlipCurrentAmps = 120.0;
    //     public static final double kSteerStatorCurrentLimitAmps = 60.0;

    //     // ======================== MOTOR GEAR RATIOS ========================
    //     public static final double kDriveGearRatio = 6.746031746031747;
    //     public static final double kSteerGearRatio = 21.428571428571427;
    //     public static final double kCouplingGearRatio = 3.5714285714285716;

    //     // ======================== WHEEL CONFIGURATION ========================
    //     public static final double kWheelRadiusInches = 2.0;

    //     // ======================== INVERSION SETTINGS ========================
    //     public static final boolean kInvertLeftSide = false;
    //     public static final boolean kInvertRightSide = true;
    //     public static final boolean kSteerMotorInverted = true;
    //     public static final boolean kEncoderInverted = false;

    //     // ======================== SIMULATION PARAMETERS ========================
    //     public static final double kSteerInertiaKgm2 = 0.01;
    //     public static final double kDriveInertiaKgm2 = 0.01;
    //     public static final double kSteerFrictionVoltage = 0.2;
    //     public static final double kDriveFrictionVoltage = 0.2;

    //     // ======================== ENCODER OFFSETS (in rotations) ========================
    //     public static final double kFrontLeftEncoderOffsetRotations = -0.0537109375;
    //     public static final double kFrontRightEncoderOffsetRotations = -0.17626953125;
    //     public static final double kBackLeftEncoderOffsetRotations = 0.397705078125;
    //     public static final double kBackRightEncoderOffsetRotations = 0.416015625;

    //     // ======================== MODULE POSITIONS ========================
    //     // Front Left Module
    //     public static final double kFrontLeftXPosInches = 13.375;
    //     public static final double kFrontLeftYPosInches = 11.375;

    //     // Front Right Module
    //     public static final double kFrontRightXPosInches = 13.375;
    //     public static final double kFrontRightYPosInches = -11.375;

    //     // Back Left Module
    //     public static final double kBackLeftXPosInches = -13.375;
    //     public static final double kBackLeftYPosInches = 11.375;

    //     // Back Right Module
    //     public static final double kBackRightXPosInches = -13.375;
    //     public static final double kBackRightYPosInches = -11.375;
    // }

    public static class Swerve {
        public static final double kPathXControllerKP = 10;
        public static final double kPathXControllerKI = 0;
        public static final double kPathXControllerKD = 0;
        public static final double kPathYControllerKP = 10;
        public static final double kPathYControllerKI = 0;
        public static final double kPathYControllerKD = 0;
        public static final double kPathThetaControllerKP = 7;
        public static final double kPathThetaControllerKI = 0;
        public static final double kPathThetaControllerKD = 0;
        public static final double kOdometryStandardDeviationX = 0.1;
        public static final double kOdometryStandardDeviationY = 0.1;
        public static final double kOdometryStandardDeviationTheta = 0.1;
        public static final double kVisionStandardDeviationX = 0.1;
        public static final double kVisionStandardDeviationY = 0.1;
        public static final double kVisionStandardDeviationTheta = 0.1;

        // ======================== STEER MOTOR GAINS ========================
        public static final double kSteerMotorKP = 100;
        public static final double kSteerMotorKI = 0;
        public static final double kSteerMotorKD = 0.5;
        public static final double kSteerMotorKS = 0.1;  // Static feedforward
        public static final double kSteerMotorKV = 2.66; // Velocity feedforward
        public static final double kSteerMotorKA = 0;    // Acceleration feedforward

        // ======================== DRIVE MOTOR GAINS ========================
        public static final double kDriveMotorKP = 0.1;
        public static final double kDriveMotorKI = 0;
        public static final double kDriveMotorKD = 0;
        public static final double kDriveMotorKS = 0;      // Static feedforward
        public static final double kDriveMotorKV = 0.124;  // Velocity feedforward

        // ======================== MOTOR CURRENT LIMITS ========================
        public static final double kSlipCurrentAmps = 120.0;
        public static final double kSteerStatorCurrentLimitAmps = 60.0;

        // ======================== MOTOR GEAR RATIOS ========================
        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kCouplingGearRatio = 3.5714285714285716;

        // ======================== WHEEL CONFIGURATION ========================
        public static final double kWheelRadiusInches = 2.0;

        // ======================== INVERSION SETTINGS ========================
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;
        public static final boolean kSteerMotorInverted = true;
        public static final boolean kEncoderInverted = false;

        // ======================== SIMULATION PARAMETERS ========================
        public static final double kSteerInertiaKgm2 = 0.01;
        public static final double kDriveInertiaKgm2 = 0.01;
        public static final double kSteerFrictionVoltage = 0.2;
        public static final double kDriveFrictionVoltage = 0.2;

        // ======================== ENCODER OFFSETS (in rotations) ========================
        public static final double kFrontLeftEncoderOffsetRotations = -0.0537109375;
        public static final double kFrontRightEncoderOffsetRotations = -0.17626953125;
        public static final double kBackLeftEncoderOffsetRotations = 0.397705078125;
        public static final double kBackRightEncoderOffsetRotations = 0.416015625;

        // ======================== MODULE POSITIONS ========================
        // Front Left Module
        public static final double kFrontLeftXPosInches = 13.375;
        public static final double kFrontLeftYPosInches = 11.375;

        // Front Right Module
        public static final double kFrontRightXPosInches = 13.375;
        public static final double kFrontRightYPosInches = -11.375;

        // Back Left Module
        public static final double kBackLeftXPosInches = -13.375;
        public static final double kBackLeftYPosInches = 11.375;

        // Back Right Module
        public static final double kBackRightXPosInches = -13.375;
        public static final double kBackRightYPosInches = -11.375;
    }

    public static class CommandSwerveDrivetrain {
        // ======================== STEER MOTOR GAINS ========================
        public static final double kSteerMotorKP = 100;
        public static final double kSteerMotorKI = 0;
        public static final double kSteerMotorKD = 0.5;
        public static final double kSteerMotorKS = 0.1;   // Static feedforward
        public static final double kSteerMotorKV = 2.66;  // Velocity feedforward
        public static final double kSteerMotorKA = 0;     // Acceleration feedforward

        // ======================== DRIVE MOTOR GAINS ========================
        public static final double kDriveMotorKP = 0.1;
        public static final double kDriveMotorKI = 0;
        public static final double kDriveMotorKD = 0;
        public static final double kDriveMotorKS = 0;       // Static feedforward
        public static final double kDriveMotorKV = 0.124;   // Velocity feedforward

        // ======================== PATH FOLLOWING GAINS FOR AUTONOMOUS ========================
        public static final double kTranslationPIDKP = 10;
        public static final double kTranslationPIDKI = 0;
        public static final double kTranslationPIDKD = 0;
        public static final double kRotationPIDKP = 7;
        public static final double kRotationPIDKI = 0;
        public static final double kRotationPIDKD = 0;

        // ======================== MOTOR CURRENT LIMITS ========================
        public static final double kSlipCurrentAmps = 120.0;
        public static final double kSteerStatorCurrentLimitAmps = 60.0;

        // ======================== MOTOR GEAR RATIOS ========================
        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kCoupleRatio = 3.5714285714285716;

        // ======================== WHEEL CONFIGURATION ========================
        public static final double kWheelRadiusInches = 2.0;
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.58);

        // ======================== INVERSION SETTINGS ========================
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;
        public static final boolean kSteerMotorInverted = true;
        public static final boolean kEncoderInverted = false;

        // ======================== SIMULATION PARAMETERS ========================
        public static final double kSteerInertiaKgm2 = 0.01;
        public static final double kDriveInertiaKgm2 = 0.01;
        public static final double kSteerFrictionVoltage = 0.2;
        public static final double kDriveFrictionVoltage = 0.2;

        // ======================== ENCODER OFFSETS (in rotations) ========================
        public static final double kFrontLeftEncoderOffsetRotations = -0.4697265625;
        public static final double kFrontRightEncoderOffsetRotations = -0.4404296875;
        public static final double kBackLeftEncoderOffsetRotations = 0.11083984375;
        public static final double kBackRightEncoderOffsetRotations = -0.111572265625;

        // ======================== MODULE POSITIONS (in inches) ========================
        // Front Left Module
        public static final double kFrontLeftXPosInches = 9.8425;
        public static final double kFrontLeftYPosInches = 9.8425;

        // Front Right Module
        public static final double kFrontRightXPosInches = 9.8425;
        public static final double kFrontRightYPosInches = -9.8425;

        // Back Left Module
        public static final double kBackLeftXPosInches = -9.8425;
        public static final double kBackLeftYPosInches = 9.8425;

        // Back Right Module
        public static final double kBackRightXPosInches = -9.8425;
        public static final double kBackRightYPosInches = -9.8425;
    }

    // ======================== AUTONOMOUS COMMAND CONSTANTS ========================

    public static class Auto {
        // Hub identification for Limelight-based distance calculations
        public static final int kHubAprilTagID = 4;
    }
}

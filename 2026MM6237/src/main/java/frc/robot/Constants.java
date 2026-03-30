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
    public static final double MaxSpeed = 2.0 * 0.35; // meters per second (limited to 35%)
    public static final double MaxAngularRate = 2 * Math.PI * 0.35; // radians per second (limited to 35%)
  }
  
    public static class Driving {
      public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts.times(0.35); // Limited to 35%
      public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1 * 0.35); // Limited to 35%
      public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class NEO {
        public static final AngularVelocity kFreeSpeed = RPM.of(5676);
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
        public static final LinearVelocity kMaxServoSpeed = Millimeters.of(60).per(Second); // Tripled from 20 — software position estimate only, does not limit physical servo speed
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
        public static final double kIntakePercentOutput = .7;
        public static final double kHomedPositionDegrees = 10;   // Home position (slightly past stowed)
        public static final double kStowedPositionDegrees = 5;   // Safe/normal stowed position
        public static final double kIntakePositionDegrees = -110; // Fully extended intake position
        public static final double kAgitatePositionDegrees = -50; // Agitate position (30° lower than previous -20°, nudges balls toward feeder)
        public static final double kAgitateIntervalSeconds = .8; // Time at each position during agitate oscillation (1/4 speed)
        public static final double kAgitateDelaySeconds = 1.25;   // Delay before agitation begins
        public static final double kPivotReduction = 50.0;
        public static final Angle kPositionTolerance = Degrees.of(5);
        public static final double kStatorCurrentLimit = 120;
        public static final double kSupplyCurrentLimit = 70;
        public static final double kPivotKP = 5;//0.1;  // Very low gain for safe testing (similar to 6% manual voltage)
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

        // ======================== RETURN SHOT CONSTANTS ========================
        // Used to lob balls back to the alliance starting zone (not into the hub).
        // Static values — no Limelight or distance detection needed.
        // Seeded from the ~3m interpolation point in the hub shot table.
        public static final double kReturnShotRPM = 3500;
        public static final double kReturnShotHoodPosition = 0.7;
    }

    public static class Limelight {
        public static final double kStandardDeviationX = 0.1;
        public static final double kStandardDeviationY = 0.1;
        public static final double kStandardDeviationTheta = 10.0;
        
        // Pipeline index for AprilTag detection (set in Limelight web interface)
        // Pipeline 1 = "HubTarget" - configured to only track hub tags (10, 26)
        public static final int kAprilTagPipelineIndex = 1;
        
        // Camera mounting parameters for distance calculation
        // TODO: Measure and update these values for your robot
        public static final double kCameraHeightMeters = 0.6;        // Height of camera lens from floor
        public static final double kCameraMountAngleDegrees = 13.0;  // Angle of camera from horizontal (positive = tilted up)
        public static final double kHubAprilTagHeightMeters = 1.2;  // Height of hub AprilTag center from floor

        // TX tolerance (degrees) for the "aimed at hub / safe to fire" LED indicator.
        // Robot must have hub tag visible AND tx within this many degrees of center.
        // Increase to make the indicator easier to trigger; decrease for tighter aim requirement.
        public static final double kAimedAtHubTxTolerance = 5.0;  // wider than command's 3° stop threshold so the LED is more forgiving
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

    // public static class Swerve {
    //     public static final double kPathXControllerKP = 10;
    //     public static final double kPathXControllerKI = 0;
    //     public static final double kPathXControllerKD = 0;
    //     public static final double kPathYControllerKP = 10;
    //     public static final double kPathYControllerKI = 0;
    //     public static final double kPathYControllerKD = 0;
    //     public static final double kPathThetaControllerKP = 7;
    //     public static final double kPathThetaControllerKI = 0;
    //     public static final double kPathThetaControllerKD = 0;
    //     public static final double kOdometryStandardDeviationX = 0.1;
    //     public static final double kOdometryStandardDeviationY = 0.1;
    //     public static final double kOdometryStandardDeviationTheta = 0.1;
    //     public static final double kVisionStandardDeviationX = 0.1;
    //     public static final double kVisionStandardDeviationY = 0.1;
    //     public static final double kVisionStandardDeviationTheta = 0.1;

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

    public static class CommandSwerveDrivetrain {
        // ======================== STEER MOTOR GAINS ========================
        public static final double kSteerMotorKP = 100;
        public static final double kSteerMotorKI = 0;
        public static final double kSteerMotorKD = 0.5;
        public static final double kSteerMotorKS = 0.1;   // Static feedforward
        public static final double kSteerMotorKV = 2.49;  // Velocity feedforward (updated for new steering motors)
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
        
        // ======================== AUTONOMOUS SPEED SCALING ========================
        // Scale factor for autonomous speeds (0.0 to 1.0)
        // Set to 0.2 for 20% speed, 1.0 for full speed
        public static final double kAutoSpeedScaleFactor = 0.2;

        // ======================== MOTOR CURRENT LIMITS ========================
        public static final double kSlipCurrentAmps = 120.0;
        public static final double kSteerStatorCurrentLimitAmps = 60.0;

        // ======================== MOTOR GEAR RATIOS ========================
        public static final double kDriveGearRatio = 6.026785714285714; // R2 Ratio = 6.03 / R1 Ratio (Slower) = 7.03
        public static final double kSteerGearRatio = 26;
        public static final double kCoupleRatio = 3.857142857142857;

        // ======================== WHEEL CONFIGURATION ========================
        public static final double kWheelRadiusInches = 2.0;
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.12 * 0.35); // Limited to 35%

        // ======================== INVERSION SETTINGS ========================
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = false;
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = false;

        // ======================== SIMULATION PARAMETERS ========================
        public static final double kSteerInertiaKgm2 = 0.01;
        public static final double kDriveInertiaKgm2 = 0.01;
        public static final double kSteerFrictionVoltage = 0.2;
        public static final double kDriveFrictionVoltage = 0.2;

        // ======================== ENCODER OFFSETS (in rotations) ========================
        public static final double kFrontLeftEncoderOffsetRotations = 0.033935546875;
        public static final double kFrontRightEncoderOffsetRotations = -0.015380859375;  // Updated from generated code
        public static final double kBackLeftEncoderOffsetRotations = 0.1416015625;      // Updated from generated code
        public static final double kBackRightEncoderOffsetRotations = 0.17333984375;    // Updated from generated code

        // ======================== MODULE POSITIONS (in inches) ========================
        // Front Left Module
        public static final double kFrontLeftXPosInches = 11.875;
        public static final double kFrontLeftYPosInches = 10.25;

        // Front Right Module
        public static final double kFrontRightXPosInches = 11.875;
        public static final double kFrontRightYPosInches = -10.25;

        // Back Left Module
        public static final double kBackLeftXPosInches = -11.875;
        public static final double kBackLeftYPosInches = 10.25;

        // Back Right Module
        public static final double kBackRightXPosInches = -11.875;
        public static final double kBackRightYPosInches = -10.25;
    }

    // ======================== AUTONOMOUS COMMAND CONSTANTS ========================

    public static class Auto {
        // Hub identification for Limelight-based distance calculations
        public static final int kRedHubAprilTagID = 10;      // Red alliance processor (center front)
        public static final int kBlueHubAprilTagID = 26;     // Blue alliance processor (center front)
        public static final int kHubAprilTagID = 4;          // Legacy - for backwards compatibility

        // All valid hub AprilTag IDs for target-lock LED indication
        // Red alliance: 8, 9, 10, 11  |  Blue alliance: 24, 25, 26, 27
        public static final int[] kHubAprilTagIDs = { 8, 9, 10, 11, 24, 25, 26, 27 };
        
        // Autonomous firing timing
        public static final double kAutoFireRunTimeSeconds = 2.0;      // Time to run feeder after shooter at speed
        public static final double kAutoPrepareAimTimeSeconds = 0.5;   // Time to attempt aiming before giving up

        // Autonomous RPM boost -- added on top of the shared interpolation table RPM
        // Compensates for auto-specific factors (battery sag, no driver correction, etc.)
        // Tunable via SmartDashboard "Auto/RPM Boost" (default 250 RPM ~ 2ft extra range)
        public static final double kAutoRpmBoostDefault = 0.0;
        
        // Autonomous roller speed -- used for intake agitation during auto fire
        // Tunable via SmartDashboard "Auto/Roller Speed %"
        public static final double kAutoRollerSpeedDefault = 0.95;
    }

    // ======================== HUB GEOMETRY (2026 REBUILT FIELD) ========================
    // All coordinates in meters, from the official 2026-rebuilt-welded.json field spec.
    // Hub is a hexagonal processor structure with 4 AprilTags per alliance hub.
    
    public static class HubGeometry {
        // --- Red Hub (far side of field, front face points toward +X / Red wall) ---
        public static final int[] kRedHubTagIDs = {8, 9, 10, 11};
        //   Tag  8: right side  (12.271, 3.431) facing -Y
        //   Tag  9: front       (12.519, 3.679) facing +X
        //   Tag 10: front       (12.519, 4.035) facing +X
        //   Tag 11: left side   (12.271, 4.638) facing +Y
        
        // Red hub center (geometric center of the hexagonal structure)
        public static final double kRedHubCenterX = 12.395;  // midpoint of front (12.519) and side (12.271) faces
        public static final double kRedHubCenterY = 4.035;   // midpoint of Y span
        
        // --- Blue Hub (near side of field, front face points toward -X / Blue wall) ---
        public static final int[] kBlueHubTagIDs = {24, 25, 26, 27};
        //   Tag 24: left side   (4.270, 4.638) facing +Y
        //   Tag 25: front       (4.022, 4.390) facing -X
        //   Tag 26: front       (4.022, 4.035) facing -X
        //   Tag 27: right side  (4.270, 3.431) facing -Y
        
        // Blue hub center (geometric center of the hexagonal structure)
        public static final double kBlueHubCenterX = 4.146;  // midpoint of front (4.022) and side (4.270) faces
        public static final double kBlueHubCenterY = 4.035;   // midpoint of Y span

        // --- Per-tag field positions (meters) ---
        // Used to compute the offset from any visible tag to its hub center.
        // Format: {x, y} for each tag, indexed by tag ID via getTagPosition()
        
        // Red hub tag positions
        public static final double[] kTag8Pos  = {12.271, 3.431};   // right side
        public static final double[] kTag9Pos  = {12.519, 3.679};   // front
        public static final double[] kTag10Pos = {12.519, 4.035};   // front
        public static final double[] kTag11Pos = {12.271, 4.638};   // left side
        
        // Blue hub tag positions
        public static final double[] kTag24Pos = {4.270, 4.638};    // left side
        public static final double[] kTag25Pos = {4.022, 4.390};    // front
        public static final double[] kTag26Pos = {4.022, 4.035};    // front
        public static final double[] kTag27Pos = {4.270, 3.431};    // right side

        // --- Per-tag TX trim offsets (degrees) ---
        // Applied AFTER the hub-center geometric correction.
        // Positive = shift aim to the right, Negative = shift aim to the left.
        // These are the initial defaults loaded into SmartDashboard at startup.
        // Tune live via TargetTuning/Trim Tag XX entries, then copy values back here.
        //
        // Mirror pairs (same structural position on hex):
        //   Tag 27 (Blue side, low Y)  ↔  Tag 8  (Red side, low Y)
        //   Tag 25 (Blue front off-ctr) ↔  Tag 9  (Red front off-ctr)
        //   Tag 26 (Blue front center)  ↔  Tag 10 (Red front center)
        //   Tag 24 (Blue side, high Y)  ↔  Tag 11 (Red side, high Y)
        // Red trims = −(blue mirror partner) because robot faces opposite direction.
        // Tag 9 field-verified at -8.0 on 2026-03-19.
        public static final double kTrimTag8  = 22.0;    // mirror of Tag27 (-8.0), negated
        public static final double kTrimTag9  = 8.0;   // mirror of Tag25 (11.0) → -11.0, FIELD TUNED to -8.0
        public static final double kTrimTag10 = 5.0;   // mirror of Tag26 (5.0), negated
        public static final double kTrimTag11 = -15.0;  // mirror of Tag24 (11.0), negated
        public static final double kTrimTag24 = 11.0;
        public static final double kTrimTag25 = 11.0;
        public static final double kTrimTag26 = 5.0;
        public static final double kTrimTag27 = -8.0;

        /**
         * Returns the default TX trim (degrees) for a given tag ID.
         */
        public static double getDefaultTrim(int tagID) {
            switch (tagID) {
                case 8:  return kTrimTag8;
                case 9:  return kTrimTag9;
                case 10: return kTrimTag10;
                case 11: return kTrimTag11;
                case 24: return kTrimTag24;
                case 25: return kTrimTag25;
                case 26: return kTrimTag26;
                case 27: return kTrimTag27;
                default: return 0.0;
            }
        }

        /**
         * Returns the field position {x, y} of a known hub tag, or null if not a hub tag.
         */
        public static double[] getTagPosition(int tagID) {
            switch (tagID) {
                case 8:  return kTag8Pos;
                case 9:  return kTag9Pos;
                case 10: return kTag10Pos;
                case 11: return kTag11Pos;
                case 24: return kTag24Pos;
                case 25: return kTag25Pos;
                case 26: return kTag26Pos;
                case 27: return kTag27Pos;
                default: return null;
            }
        }

        /**
         * Returns the hub center {x, y} for the hub that the given tag belongs to,
         * or null if not a hub tag.
         */
        public static double[] getHubCenter(int tagID) {
            switch (tagID) {
                case 8: case 9: case 10: case 11:
                    return new double[] {kRedHubCenterX, kRedHubCenterY};
                case 24: case 25: case 26: case 27:
                    return new double[] {kBlueHubCenterX, kBlueHubCenterY};
                default:
                    return null;
            }
        }

        /**
         * Returns true if the given tag ID belongs to any hub (red or blue).
         */
        public static boolean isHubTag(int tagID) {
            return getTagPosition(tagID) != null;
        }

        /**
         * Returns true if the given tag belongs to the specified alliance's hub.
         */
        public static boolean isOurHubTag(int tagID, edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
            if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                return tagID >= 8 && tagID <= 11;
            } else {
                return tagID >= 24 && tagID <= 27;
            }
        }
    }
}

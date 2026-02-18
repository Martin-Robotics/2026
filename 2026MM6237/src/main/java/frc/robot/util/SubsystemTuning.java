package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * SubsystemTuning provides centralized SmartDashboard displays and tunable parameters
 * for all subsystems. This allows operators to view real-time data and adjust tuning
 * values without recompiling code.
 * 
 * Usage:
 *   - Call initializeDashboards() in Robot.robotInit() or RobotContainer
 *   - Call updateDashboards() periodically in Robot.robotPeriodic()
 *   - Call applyTuning() periodically to read dashboard values and apply them
 */
public class SubsystemTuning {
    
    // ======================== FEEDER TUNING ========================
    public static class FeederTuning {
        private static double dashboardFeedRPM = Constants.Feeder.kFeedRPM;
        private static double dashboardTestVoltage = 0.3;
        
        public static void initialize() {
            SmartDashboard.putNumber("Feeder/Target RPM", dashboardFeedRPM);
            SmartDashboard.putNumber("Feeder/Test Voltage %", dashboardTestVoltage);
            SmartDashboard.putBoolean("Feeder/Initialized", true);
        }
        
        public static void update(Feeder feeder) {
            // Display current status
            SmartDashboard.putNumber("Feeder/Current RPM", feeder.getCurrentRPM());
            SmartDashboard.putNumber("Feeder/Stator Current (A)", feeder.getStatorCurrent());
            SmartDashboard.putNumber("Feeder/Supply Current (A)", feeder.getSupplyCurrent());
            
            // Read tunable values from dashboard
            dashboardFeedRPM = SmartDashboard.getNumber("Feeder/Target RPM", dashboardFeedRPM);
            dashboardTestVoltage = SmartDashboard.getNumber("Feeder/Test Voltage %", dashboardTestVoltage);
        }
        
        public static double getTargetRPM() {
            return dashboardFeedRPM;
        }
        
        public static double getTestVoltagePercent() {
            return dashboardTestVoltage;
        }
    }
    
    // ======================== SHOOTER TUNING ========================
    public static class ShooterTuning {
        private static double dashboardTargetRPM = 0.0;
        private static double dashboardTestVoltage = 0.3;
        private static double dashboardLeftKP = Constants.Shooter.kLeftKP;
        private static double dashboardKI = Constants.Shooter.kKI;
        private static double dashboardKD = Constants.Shooter.kKD;
        
        public static void initialize() {
            SmartDashboard.putNumber("Shooter/Target RPM", dashboardTargetRPM);
            SmartDashboard.putNumber("Shooter/Test Voltage %", dashboardTestVoltage);
            SmartDashboard.putNumber("Shooter/Left KP", dashboardLeftKP);
            SmartDashboard.putNumber("Shooter/KI", dashboardKI);
            SmartDashboard.putNumber("Shooter/KD", dashboardKD);
            SmartDashboard.putBoolean("Shooter/Initialized", true);
        }
        
        public static void update(Shooter shooter) {
            // Display current status
            SmartDashboard.putNumber("Shooter/Left Motor RPM", shooter.getLeftMotorRPM());
            SmartDashboard.putNumber("Shooter/Middle Motor RPM", shooter.getMiddleMotorRPM());
            SmartDashboard.putNumber("Shooter/Right Motor RPM", shooter.getRightMotorRPM());
            SmartDashboard.putNumber("Shooter/Left Stator Current (A)", shooter.getLeftStatorCurrent());
            SmartDashboard.putNumber("Shooter/Middle Stator Current (A)", shooter.getMiddleStatorCurrent());
            SmartDashboard.putNumber("Shooter/Right Stator Current (A)", shooter.getRightStatorCurrent());
            
            // Read tunable values from dashboard
            dashboardTargetRPM = SmartDashboard.getNumber("Shooter/Target RPM", dashboardTargetRPM);
            dashboardTestVoltage = SmartDashboard.getNumber("Shooter/Test Voltage %", dashboardTestVoltage);
            dashboardLeftKP = SmartDashboard.getNumber("Shooter/Left KP", dashboardLeftKP);
            dashboardKI = SmartDashboard.getNumber("Shooter/KI", dashboardKI);
            dashboardKD = SmartDashboard.getNumber("Shooter/KD", dashboardKD);
        }
        
        public static double getTargetRPM() {
            return dashboardTargetRPM;
        }
        
        public static double getTestVoltagePercent() {
            return dashboardTestVoltage;
        }
        
        public static double getLeftKP() {
            return dashboardLeftKP;
        }
        
        public static double getKI() {
            return dashboardKI;
        }
        
        public static double getKD() {
            return dashboardKD;
        }
    }
    
    // ======================== INTAKE TUNING ========================
    public static class IntakeTuning {
        private static double dashboardIntakePercentOutput = Constants.Intake.kIntakePercentOutput;
        private static double dashboardTestVoltage = 0.3;
        private static double dashboardPivotKP = Constants.Intake.kPivotKP;
        private static double dashboardPivotKI = Constants.Intake.kPivotKI;
        private static double dashboardPivotKD = Constants.Intake.kPivotKD;
        private static double dashboardHomingPercentOutput = Constants.Intake.kHomingPercentOutput;
        
        public static void initialize() {
            SmartDashboard.putNumber("Intake/Intake Speed %", dashboardIntakePercentOutput);
            SmartDashboard.putNumber("Intake/Test Voltage %", dashboardTestVoltage);
            SmartDashboard.putNumber("Intake/Pivot KP", dashboardPivotKP);
            SmartDashboard.putNumber("Intake/Pivot KI", dashboardPivotKI);
            SmartDashboard.putNumber("Intake/Pivot KD", dashboardPivotKD);
            SmartDashboard.putNumber("Intake/Homing Voltage %", dashboardHomingPercentOutput);
            SmartDashboard.putBoolean("Intake/Initialized", true);
        }
        
        public static void update(Intake intake) {
            // Display current status
            SmartDashboard.putNumber("Intake/Pivot Angle (degrees)", intake.getPivotAngleDegrees());
            SmartDashboard.putNumber("Intake/Roller RPM", intake.getRollerRPM());
            SmartDashboard.putNumber("Intake/Pivot Stator Current (A)", intake.getPivotStatorCurrent());
            SmartDashboard.putNumber("Intake/Roller Stator Current (A)", intake.getRollerStatorCurrent());
            SmartDashboard.putBoolean("Intake/Is Homed", intake.isHomed());
            
            // Read tunable values from dashboard
            dashboardIntakePercentOutput = SmartDashboard.getNumber("Intake/Intake Speed %", dashboardIntakePercentOutput);
            dashboardTestVoltage = SmartDashboard.getNumber("Intake/Test Voltage %", dashboardTestVoltage);
            dashboardPivotKP = SmartDashboard.getNumber("Intake/Pivot KP", dashboardPivotKP);
            dashboardPivotKI = SmartDashboard.getNumber("Intake/Pivot KI", dashboardPivotKI);
            dashboardPivotKD = SmartDashboard.getNumber("Intake/Pivot KD", dashboardPivotKD);
            dashboardHomingPercentOutput = SmartDashboard.getNumber("Intake/Homing Voltage %", dashboardHomingPercentOutput);
        }
        
        public static double getIntakePercentOutput() {
            return dashboardIntakePercentOutput;
        }
        
        public static double getTestVoltagePercent() {
            return dashboardTestVoltage;
        }
        
        public static double getPivotKP() {
            return dashboardPivotKP;
        }
        
        public static double getPivotKI() {
            return dashboardPivotKI;
        }
        
        public static double getPivotKD() {
            return dashboardPivotKD;
        }
        
        public static double getHomingPercentOutput() {
            return dashboardHomingPercentOutput;
        }
    }
    
    // ======================== HOOD TUNING ========================
    public static class HoodTuning {
        private static double dashboardMinPosition = Constants.Hood.kMinPosition;
        private static double dashboardMaxPosition = Constants.Hood.kMaxPosition;
        private static double dashboardPositionTolerance = Constants.Hood.kPositionTolerance;
        private static double dashboardTestPosition = Constants.Hood.kInitialPosition;
        
        public static void initialize() {
            SmartDashboard.putNumber("Hood/Min Position", dashboardMinPosition);
            SmartDashboard.putNumber("Hood/Max Position", dashboardMaxPosition);
            SmartDashboard.putNumber("Hood/Position Tolerance", dashboardPositionTolerance);
            SmartDashboard.putNumber("Hood/Test Position", dashboardTestPosition);
            SmartDashboard.putBoolean("Hood/Initialized", true);
        }
        
        public static void update(Hood hood) {
            // Display current status
            SmartDashboard.putNumber("Hood/Current Position", hood.getCurrentPosition());
            SmartDashboard.putNumber("Hood/Target Position", hood.getTargetPosition());
            SmartDashboard.putBoolean("Hood/Within Tolerance", hood.isPositionWithinTolerance());
            
            // Read tunable values from dashboard
            dashboardMinPosition = SmartDashboard.getNumber("Hood/Min Position", dashboardMinPosition);
            dashboardMaxPosition = SmartDashboard.getNumber("Hood/Max Position", dashboardMaxPosition);
            dashboardPositionTolerance = SmartDashboard.getNumber("Hood/Position Tolerance", dashboardPositionTolerance);
            dashboardTestPosition = SmartDashboard.getNumber("Hood/Test Position", dashboardTestPosition);
        }
        
        public static double getMinPosition() {
            return dashboardMinPosition;
        }
        
        public static double getMaxPosition() {
            return dashboardMaxPosition;
        }
        
        public static double getPositionTolerance() {
            return dashboardPositionTolerance;
        }
        
        public static double getTestPosition() {
            return dashboardTestPosition;
        }
    }
    
    // ======================== HANGER TUNING ========================
    public static class HangerTuning {
        private static double dashboardTestVoltage = 0.3;
        private static double dashboardKP = Constants.Hanger.kP;
        private static double dashboardKI = Constants.Hanger.kI;
        private static double dashboardKD = Constants.Hanger.kD;
        private static double dashboardHomingPercentOutput = Constants.Hanger.kHomingPercentOutput;
        private static double dashboardHomingCurrentThreshold = Constants.Hanger.kHomingCurrentThreshold;
        
        public static void initialize() {
            SmartDashboard.putNumber("Hanger/Test Voltage %", dashboardTestVoltage);
            SmartDashboard.putNumber("Hanger/KP", dashboardKP);
            SmartDashboard.putNumber("Hanger/KI", dashboardKI);
            SmartDashboard.putNumber("Hanger/KD", dashboardKD);
            SmartDashboard.putNumber("Hanger/Homing Voltage %", dashboardHomingPercentOutput);
            SmartDashboard.putNumber("Hanger/Homing Current Threshold (A)", dashboardHomingCurrentThreshold);
            SmartDashboard.putBoolean("Hanger/Initialized", true);
        }
        
        public static void update(Hanger hanger) {
            // Display current status
            SmartDashboard.putNumber("Hanger/Extension (inches)", hanger.getCurrentExtensionInches());
            SmartDashboard.putNumber("Hanger/Supply Current (A)", hanger.getSupplyCurrent());
            SmartDashboard.putBoolean("Hanger/Is Homed", hanger.isHomed());
            
            // Read tunable values from dashboard
            dashboardTestVoltage = SmartDashboard.getNumber("Hanger/Test Voltage %", dashboardTestVoltage);
            dashboardKP = SmartDashboard.getNumber("Hanger/KP", dashboardKP);
            dashboardKI = SmartDashboard.getNumber("Hanger/KI", dashboardKI);
            dashboardKD = SmartDashboard.getNumber("Hanger/KD", dashboardKD);
            dashboardHomingPercentOutput = SmartDashboard.getNumber("Hanger/Homing Voltage %", dashboardHomingPercentOutput);
            dashboardHomingCurrentThreshold = SmartDashboard.getNumber("Hanger/Homing Current Threshold (A)", dashboardHomingCurrentThreshold);
        }
        
        public static double getTestVoltagePercent() {
            return dashboardTestVoltage;
        }
        
        public static double getKP() {
            return dashboardKP;
        }
        
        public static double getKI() {
            return dashboardKI;
        }
        
        public static double getKD() {
            return dashboardKD;
        }
        
        public static double getHomingPercentOutput() {
            return dashboardHomingPercentOutput;
        }
        
        public static double getHomingCurrentThreshold() {
            return dashboardHomingCurrentThreshold;
        }
    }
    
    // ======================== FLOOR TUNING ========================
    public static class FloorTuning {
        private static double dashboardFeedPercentOutput = Constants.Floor.kFeedPercentOutput;
        private static double dashboardTestVoltage = 0.3;
        
        public static void initialize() {
            SmartDashboard.putNumber("Floor/Feed Speed %", dashboardFeedPercentOutput);
            SmartDashboard.putNumber("Floor/Test Voltage %", dashboardTestVoltage);
            SmartDashboard.putBoolean("Floor/Initialized", true);
        }
        
        public static void update(Floor floor) {
            // Display current status
            SmartDashboard.putNumber("Floor/Current RPM", floor.getCurrentRPM());
            SmartDashboard.putNumber("Floor/Stator Current (A)", floor.getStatorCurrent());
            SmartDashboard.putNumber("Floor/Supply Current (A)", floor.getSupplyCurrent());
            
            // Read tunable values from dashboard
            dashboardFeedPercentOutput = SmartDashboard.getNumber("Floor/Feed Speed %", dashboardFeedPercentOutput);
            dashboardTestVoltage = SmartDashboard.getNumber("Floor/Test Voltage %", dashboardTestVoltage);
        }
        
        public static double getFeedPercentOutput() {
            return dashboardFeedPercentOutput;
        }
        
        public static double getTestVoltagePercent() {
            return dashboardTestVoltage;
        }
    }
    
    // ======================== MASTER METHODS ========================
    
    /**
     * Initialize all tuning dashboards. Call this once at startup.
     */
    public static void initializeAllDashboards() {
        SmartDashboard.putString("Subsystem Tuning/Status", "Initializing...");
        
        FeederTuning.initialize();
        ShooterTuning.initialize();
        IntakeTuning.initialize();
        HoodTuning.initialize();
        HangerTuning.initialize();
        FloorTuning.initialize();
        
        SmartDashboard.putString("Subsystem Tuning/Status", "Ready");
    }
    
    /**
     * Update all subsystem displays. Call this periodically in robotPeriodic().
     */
    public static void updateAllDashboards(
            Feeder feeder,
            Shooter shooter,
            Intake intake,
            Hood hood,
            Hanger hanger,
            Floor floor) {
        
        FeederTuning.update(feeder);
        ShooterTuning.update(shooter);
        IntakeTuning.update(intake);
        HoodTuning.update(hood);
        HangerTuning.update(hanger);
        FloorTuning.update(floor);
    }
}

/**
 * EXAMPLE COMMAND CODE USING SUBSYSTEM TUNING VALUES
 * 
 * This file demonstrates how to use the tuning values from SubsystemTuning
 * in your command code. Copy and adapt these examples to your commands.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Floor;
import frc.robot.util.SubsystemTuning;

public class TuningExampleCommands {

    // ======================== FEEDER EXAMPLES ========================

    /**
     * Feed using the dashboard-tuned RPM value
     */
    public static Command feederFeedDashboardRPMCommand(Feeder feeder) {
        return feeder.runEnd(
            () -> {
                // Use dashboard test voltage for safe testing
                feeder.setPercentOutput(SubsystemTuning.FeederTuning.getTestVoltagePercent());
            },
            () -> feeder.setPercentOutput(0.0)
        ).withName("Feeder Feed (Dashboard RPM)");
    }

    // ======================== SHOOTER EXAMPLES ========================

    /**
     * Spin up shooter using dashboard target RPM
     */
    public static Command shooterSpinUpCommand(Shooter shooter) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                double targetRPM = SubsystemTuning.ShooterTuning.getTargetRPM();
                if (targetRPM > 0) {
                    shooter.setRPM(targetRPM);
                }
            }),
            Commands.waitUntil(shooter::isVelocityWithinTolerance)
        ).withName("Shooter Spin Up (Tuned)");
    }

    /**
     * Test shooter with dashboard voltage percentage
     */
    public static Command shooterTestCommand(Shooter shooter) {
        return shooter.runEnd(
            () -> {
                double testVoltagePercent = SubsystemTuning.ShooterTuning.getTestVoltagePercent();
                shooter.setPercentOutput(testVoltagePercent);
            },
            shooter::stop
        ).withName("Shooter Test (Dashboard Voltage)");
    }

    // ======================== INTAKE EXAMPLES ========================

    /**
     * Run intake with dashboard-tuned speed
     */
    public static Command intakeDashboardCommand(Intake intake) {
        return intake.runEnd(
            () -> {
                // Use dashboard tuned speed
                intake.set(Intake.Speed.INTAKE);
            },
            () -> intake.set(Intake.Speed.STOP)
        ).withName("Intake (Dashboard Speed)");
    }

    /**
     * Home intake using dashboard homing voltage
     */
    public static Command intakeHomeCommand(Intake intake) {
        return intake.homingCommand().withName("Intake Home (Dashboard)");
    }

    // ======================== HOOD EXAMPLES ========================

    /**
     * Move hood to dashboard test position
     */
    public static Command hoodTestPositionCommand(Hood hood) {
        return hood.runOnce(() -> {
            double testPosition = SubsystemTuning.HoodTuning.getTestPosition();
            hood.setPosition(testPosition);
        })
        .andThen(Commands.waitUntil(hood::isPositionWithinTolerance))
        .withName("Hood Test Position");
    }

    /**
     * Move hood to specific position and wait, using dashboard tolerance
     */
    public static Command hoodPositionCommand(Hood hood, double targetPosition) {
        return hood.runOnce(() -> hood.setPosition(targetPosition))
            .andThen(Commands.waitUntil(() -> {
                double tolerance = SubsystemTuning.HoodTuning.getPositionTolerance();
                double current = hood.getCurrentPosition();
                return Math.abs(current - targetPosition) <= tolerance;
            }))
            .withName("Hood To Position " + targetPosition);
    }

    // ======================== HANGER EXAMPLES ========================

    /**
     * Extend hanger using dashboard test voltage
     */
    public static Command hangerExtendCommand(Hanger hanger) {
        return hanger.runEnd(
            () -> {
                double testVoltage = SubsystemTuning.HangerTuning.getTestVoltagePercent();
                hanger.setPercentOutput(testVoltage);
            },
            () -> hanger.setPercentOutput(0.0)
        ).withName("Hanger Extend (Dashboard Voltage)");
    }

    /**
     * Retract hanger using dashboard test voltage (negative)
     */
    public static Command hangerRetractCommand(Hanger hanger) {
        return hanger.runEnd(
            () -> {
                double testVoltage = SubsystemTuning.HangerTuning.getTestVoltagePercent();
                hanger.setPercentOutput(-testVoltage);
            },
            () -> hanger.setPercentOutput(0.0)
        ).withName("Hanger Retract (Dashboard Voltage)");
    }

    // ======================== FLOOR EXAMPLES ========================

    /**
     * Feed with dashboard-tuned speed
     */
    public static Command floorFeedCommand(Floor floor) {
        return floor.feedCommand().withName("Floor Feed (Dashboard)");
    }

    // ======================== MULTI-SUBSYSTEM EXAMPLES ========================

    /**
     * Example: Prepare to shoot
     * Uses dashboard values for shooter RPM and hood position
     */
    public static Command prepareToShootCommand(
            Shooter shooter,
            Hood hood,
            Feeder feeder) {
        
        return Commands.parallel(
            // Spin up shooter to dashboard RPM
            Commands.runOnce(() -> {
                double targetRPM = SubsystemTuning.ShooterTuning.getTargetRPM();
                if (targetRPM > 0) {
                    shooter.setRPM(targetRPM);
                }
            })
            .andThen(Commands.waitUntil(shooter::isVelocityWithinTolerance)),
            
            // Move hood to test position
            Commands.runOnce(() -> {
                double testPosition = SubsystemTuning.HoodTuning.getTestPosition();
                hood.setPosition(testPosition);
            })
            .andThen(Commands.waitUntil(hood::isPositionWithinTolerance))
        ).withName("Prepare to Shoot (Tuned)");
    }

    /**
     * Example: Full intake sequence
     * Uses dashboard homing voltage and intake speed
     */
    public static Command intakeSequenceCommand(Intake intake, Floor floor) {
        return Commands.sequence(
            // Home intake if needed (using conditional operator)
            intake.isHomed() ? Commands.none() : intake.homingCommand(),
            
            // Move to intake position
            Commands.runOnce(() -> intake.set(Intake.Position.INTAKE)),
            
            // Run intake with dashboard speed
            Commands.runOnce(() -> intake.set(Intake.Speed.INTAKE)),
            
            // Run floor as well
            Commands.runOnce(() -> floor.set(Floor.Speed.FEED))
        ).withName("Intake Sequence (Tuned)");
    }

    // ======================== TUNING AND VALIDATION COMMANDS ========================

    /**
     * Validate that all dashboard values are reasonable
     * Returns true if all values are in expected ranges
     */
    public static boolean validateDashboardTuning() {
        boolean valid = true;
        
        // Feeder checks
        if (SubsystemTuning.FeederTuning.getTargetRPM() < 0 || SubsystemTuning.FeederTuning.getTargetRPM() > 6000) {
            System.out.println("WARNING: Feeder Target RPM out of range");
            valid = false;
        }
        
        if (SubsystemTuning.FeederTuning.getTestVoltagePercent() < 0 || SubsystemTuning.FeederTuning.getTestVoltagePercent() > 1.0) {
            System.out.println("WARNING: Feeder Test Voltage out of range");
            valid = false;
        }
        
        // Shooter checks
        if (SubsystemTuning.ShooterTuning.getTargetRPM() < 0 || SubsystemTuning.ShooterTuning.getTargetRPM() > 6000) {
            System.out.println("WARNING: Shooter Target RPM out of range");
            valid = false;
        }
        
        // Hood checks
        if (SubsystemTuning.HoodTuning.getMinPosition() < 0 || SubsystemTuning.HoodTuning.getMinPosition() > 1.0) {
            System.out.println("WARNING: Hood Min Position out of range");
            valid = false;
        }
        
        if (SubsystemTuning.HoodTuning.getMaxPosition() < 0 || SubsystemTuning.HoodTuning.getMaxPosition() > 1.0) {
            System.out.println("WARNING: Hood Max Position out of range");
            valid = false;
        }
        
        return valid;
    }

    /**
     * Print all current tuning values to console
     */
    public static void printTuningValues() {
        System.out.println("=== CURRENT TUNING VALUES ===");
        System.out.println("Feeder Target RPM: " + SubsystemTuning.FeederTuning.getTargetRPM());
        System.out.println("Feeder Test Voltage %: " + SubsystemTuning.FeederTuning.getTestVoltagePercent());
        System.out.println("Shooter Target RPM: " + SubsystemTuning.ShooterTuning.getTargetRPM());
        System.out.println("Shooter Test Voltage %: " + SubsystemTuning.ShooterTuning.getTestVoltagePercent());
        System.out.println("Shooter KP: " + SubsystemTuning.ShooterTuning.getLeftKP());
        System.out.println("Intake Speed %: " + SubsystemTuning.IntakeTuning.getIntakePercentOutput());
        System.out.println("Intake Homing Voltage %: " + SubsystemTuning.IntakeTuning.getHomingPercentOutput());
        System.out.println("Hood Test Position: " + SubsystemTuning.HoodTuning.getTestPosition());
        System.out.println("Hood Min/Max: " + SubsystemTuning.HoodTuning.getMinPosition() + " / " + SubsystemTuning.HoodTuning.getMaxPosition());
        System.out.println("Hanger Test Voltage %: " + SubsystemTuning.HangerTuning.getTestVoltagePercent());
        System.out.println("Hanger KP/KI/KD: " + SubsystemTuning.HangerTuning.getKP() + " / " + SubsystemTuning.HangerTuning.getKI() + " / " + SubsystemTuning.HangerTuning.getKD());
        System.out.println("Floor Feed Speed %: " + SubsystemTuning.FloorTuning.getFeedPercentOutput());
    }
}

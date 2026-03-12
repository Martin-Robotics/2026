package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  // The Blinkin acts like a Spark motor controller via PWM
  private final Spark m_blinkin;

  // Limelight target-lock state
  private LimelightSubsystem6237 m_limelight = null;
  private double m_basePattern = Patterns.TELEOP;

  public LEDSubsystem() {
    // Port 2 matches the Rio's PWM port
    m_blinkin = new Spark(2);
  }

  /**
   * Provides the limelight subsystem so the LEDs can indicate target lock.
   * Call this from RobotContainer after both subsystems are constructed.
   */
  public void setLimelightSubsystem(LimelightSubsystem6237 limelight) {
    m_limelight = limelight;
  }

  /**
   * Sets the Blinkin to a specific pattern.
   * Values range from -1.0 to 1.0.
   * See the REV Blinkin manual for the pattern table.
   * This pattern is used as the "base" when no limelight target is locked.
   */
  public void setPattern(double patternValue) {
    m_basePattern = patternValue;
    m_blinkin.set(patternValue);
  }

  @Override
  public void periodic() {
    if (m_limelight != null && m_limelight.isAimedAtHub()) {
      // Color Waves, Forest palette when robot is aimed at the hub (safe to fire)
      m_blinkin.set(Patterns.TARGET_LOCKED);
    } else {
      // No target — restore whatever pattern was last set externally
      m_blinkin.set(m_basePattern);
    }
  }

  // Pre-defined pattern constants for the 2026 game REBUILT
  public static final class Patterns {
    public static final double TELEOP = 0.81;        // Aqua
    public static final double AUTONOMOUS = 0.89;    // Blue Violet
    public static final double ACTIVE_HUB = 0.75;   // Dark Green
    public static final double INACTIVE_HUB = 0.59; // Dark Red
    public static final double WARNING = 0.56;       // Hot Pink (3s warning before hub deactivates)
    public static final double END_GAME = -0.87;     // Confetti -- Last 30 Seconds of match
    public static final double TARGET_LOCKED = -0.37; // Color Waves, Forest (limelight target lock)
  }
}

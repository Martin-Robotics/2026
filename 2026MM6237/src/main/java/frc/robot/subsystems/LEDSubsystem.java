package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  // The Blinkin acts like a Spark motor controller via PWM
  private final Spark m_blinkin;

  public LEDSubsystem() {
    // Port 2 matches the Rio's PWM port
    m_blinkin = new Spark(2);
  }

  /**
   * Sets the Blinkin to a specific pattern.
   * Values range from -1.0 to 1.0.
   * See the REV Blinkin manual for the pattern table.
   */
  public void setPattern(double patternValue) {
    m_blinkin.set(patternValue);
  }

  // Pre-defined pattern constants for the 2026 game REBUILT
  public static final class Patterns {
    public static final double TELEOP = -0.11;      //Strobe Gold
    public static final double AUTONOMOUS = 0.87;   //Gold
    public static final double ACTIVE_HUB = 0.77;   // Green
    public static final double INACTIVE_HUB = 0.61; // Red
    public static final double WARNING = -0.07;     // Strobe Red (3s warning before hub deactivates)
    public static final double END_GAME = 0.99;    // Violet / Rainbow
  }
}

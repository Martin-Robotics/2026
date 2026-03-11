package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubMonitor extends SubsystemBase {
  private boolean isOurHubActive = true;
  private String activeHubLabel = "Both";
  private double hubCountdown = 0.0;
  private String hubCountdownLabel = "";

  @Override
  public void periodic() {
    // 1. Get Game Data: FMS sends this at the start of TELEOP 
    // Common format: "R" (Red starts inactive) or "B" (Blue starts inactive)
    String gameData = DriverStation.getGameSpecificMessage();
    
    // 2. Get Match Time (Counts down from 2:40/160s in REBUILT) 
    double matchTime = Timer.getMatchTime();
    
    // 3. Determine Active Status based on Table 6-3 
    if (matchTime > 130 || matchTime < 30) {
      // AUTO, TRANSITION, and END GAME - Both HUBS are ALWAYS active 
      isOurHubActive = true;
      activeHubLabel = "Both";
      if (matchTime > 130) {
        // Shifts haven't started yet - countdown to when shifts begin
        hubCountdown = matchTime - 130;
        hubCountdownLabel = "Shifts start in";
      } else {
        // Endgame - both active until match ends
        hubCountdown = matchTime;
        hubCountdownLabel = "Active until end";
      }
    } else {
      // Calculate shifts (130s down to 30s) 
      isOurHubActive = calculateShiftStatus(matchTime, gameData);
      activeHubLabel = calculateActiveHubLabel(matchTime, gameData);
      hubCountdown = calculateOurHubCountdown(matchTime, gameData);
      hubCountdownLabel = isOurHubActive ? "Active for" : "Inactive for";
    }

    // Publish to SmartDashboard
    SmartDashboard.putString("Hub/Active Hub", activeHubLabel);
    SmartDashboard.putBoolean("Hub/Our Hub Active", isOurHubActive);
    SmartDashboard.putString("Hub/Status", hubCountdownLabel);
    SmartDashboard.putNumber("Hub/Countdown (s)", Math.round(hubCountdown * 10.0) / 10.0);
    SmartDashboard.putNumber("Match Time (s)", Math.round(matchTime * 10.0) / 10.0);
  }

  /**
   * Calculate the countdown relevant to OUR alliance:
   * - If our hub IS active: time until it becomes inactive (end of our active shift, 
   *   or until the "Both" zone at 30s if we're active in the last shift)
   * - If our hub is NOT active: time until it becomes active again (end of current shift)
   */
  private double calculateOurHubCountdown(double matchTime, String data) {
    if (data == null || data.isEmpty()) return 0.0;

    // Shift boundaries (match time counting down): 130, 105, 80, 55, 30
    double[] boundaries = {105, 80, 55, 30};

    // Find the next boundary below current match time (next shift change)
    double nextBoundary = 30; // default: end of shift zone
    for (double b : boundaries) {
      if (matchTime > b) {
        nextBoundary = b;
        break;
      }
    }

    if (isOurHubActive) {
      // Our hub is active. We want to know when it goes INACTIVE.
      // Check if it will go inactive at the next boundary, or if we stay active
      // because both hubs become active at 30s.
      if (nextBoundary == 30) {
        // We're in the last shift (55s-30s). At 30s both hubs go active,
        // so our hub never goes inactive again this match.
        return matchTime - 30;
      }
      // At the next boundary, the shift flips. Our hub goes inactive.
      return matchTime - nextBoundary;
    } else {
      // Our hub is inactive. Next boundary = we become active again.
      return matchTime - nextBoundary;
    }
  }

  /**
   * Determine which alliance's hub is currently active as a display string.
   */
  private String calculateActiveHubLabel(double time, String data) {
    if (data == null || data.isEmpty()) return "Both";

    boolean isOddShift = ((int)((130 - time) / 25) % 2 == 0);
    boolean redStartsActive = !data.equals("R");

    boolean redActive;
    if (isOddShift) {
      redActive = redStartsActive;
    } else {
      redActive = !redStartsActive;
    }

    return redActive ? "Red" : "Blue";
  }

  private boolean calculateShiftStatus(double time, String data) {
    if (data == null || data.isEmpty()) return true;

    // Logic for 25-second ALLIANCE SHIFTS 
    // Shift 1: 130s - 105s (2:10 - 1:45)
    // Shift 2: 105s - 80s  (1:45 - 1:20)
    // Shift 3: 80s - 55s   (1:20 - 0:55)
    // Shift 4: 55s - 30s   (0:55 - 0:30)
    
    // Simplification: Check if we are in an odd or even shift
    boolean isOddShift = ((int)((130 - time) / 25) % 2 == 0);
    
    // If gameData is 'R', Red starts inactive (Odd shifts), Blue starts active
    boolean redStartsActive = !data.equals("R");
    
    // Adjust based on your current alliance
    boolean weAreRed = (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red);
    
    if (isOddShift) {
        return (weAreRed == redStartsActive);
    } else {
        return (weAreRed != redStartsActive);
    }
  }

  public boolean isHubActive() {
    return isOurHubActive;
  }
}
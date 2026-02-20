package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubMonitor extends SubsystemBase {
  private boolean isOurHubActive = true;

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
    } else {
      // Calculate shifts (130s down to 30s) 
      isOurHubActive = calculateShiftStatus(matchTime, gameData);
    }

    SmartDashboard.putBoolean("Our Hub Active", isOurHubActive);
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
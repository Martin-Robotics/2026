# SmartDashboard Tuning System - Deployment Checklist

## Pre-Deployment

- [ ] All code compiles without errors
- [ ] Run `./gradlew build` successfully
- [ ] No unused import warnings that are concerning
- [ ] Robot.java has SubsystemTuning import
- [ ] Robot.robotPeriodic() includes SubsystemTuning update call
- [ ] RobotContainer has SubsystemTuning initialization
- [ ] RobotContainer has all subsystem getter methods
- [ ] All subsystem getter methods are public
- [ ] OperatorController is imported in RobotContainer
- [ ] OperatorController.mapXboxController() is called in configureBindings()

## First Test - Robot Enabled

- [ ] Connect second Xbox controller (Port 1)
- [ ] SmartDashboard opens without errors
- [ ] All subsystem tabs appear under "SmartDashboard"
- [ ] Tuning subsystem shows "Ready" status
- [ ] At least one read-only value is updating (e.g., Shooter RPM)

## Operator Controller Testing

For each subsystem, verify button responses:

### Feeder
- [ ] LB held: Motor spins forward, RPM increases above 0
- [ ] Release LB: Motor stops, RPM returns to 0
- [ ] LT held: Motor spins reverse (if wired reversible)
- [ ] Release LT: Motor stops

### Shooter
- [ ] RB held: All three motors spin, RPM values increase
- [ ] Release RB: Motors coast/brake, RPM decreases
- [ ] RT held: Motors reverse (if reversible)
- [ ] Release RT: Motors stop
- [ ] All three motors RPM values update independently

### Intake
- [ ] Y held: Intake runs forward (rollers), pivot moves, currents increase
- [ ] Release Y: Everything stops
- [ ] X held: Intake stows (stop motion)
- [ ] B held: Agitate sequence runs (or starts if sequence-based)
- [ ] A held: Homing sequence runs
- [ ] "Is Homed" indicator changes after homing

### Hood
- [ ] Move left stick forward: Hood position increases
- [ ] Move left stick backward: Hood position decreases
- [ ] Return stick to neutral: Hood position holds steady
- [ ] Position stays within min/max bounds

### Hanger
- [ ] DPad ↑: Extension increases, current increases
- [ ] Release: Extension holds, current drops
- [ ] DPad ↓: Extension decreases
- [ ] "Is Homed" indicator shows status

### Floor
- [ ] DPad ←: Motor spins at feed speed, RPM increases
- [ ] Release: Motor stops
- [ ] DPad →: Motor stops/no reverse (as designed)

## SmartDashboard Verification

For each subsystem tab, verify displays appear:

### Feeder Tab
- [ ] Current RPM display (read-only)
- [ ] Stator Current display (read-only)
- [ ] Supply Current display (read-only)
- [ ] Target RPM slider (editable)
- [ ] Test Voltage % slider (editable)

### Shooter Tab
- [ ] Left/Middle/Right Motor RPM (all three separate)
- [ ] Left/Middle/Right Stator Current (all three separate)
- [ ] Target RPM slider
- [ ] Test Voltage %, KP, KI, KD sliders

### Intake Tab
- [ ] Pivot Angle display
- [ ] Roller RPM display
- [ ] Pivot/Roller Stator Current displays
- [ ] Is Homed indicator
- [ ] All tuning sliders present

### Hood Tab
- [ ] Current/Target Position displays
- [ ] Within Tolerance indicator
- [ ] Min/Max Position sliders
- [ ] Position Tolerance slider
- [ ] Test Position slider

### Hanger Tab
- [ ] Extension display
- [ ] Supply Current display
- [ ] Is Homed indicator
- [ ] Test Voltage, KP, KI, KD, Homing sliders

### Floor Tab
- [ ] Current RPM display
- [ ] Stator Current display
- [ ] Supply Current display
- [ ] Feed Speed %, Test Voltage % sliders

## Current Draw Validation

For each motor, verify current values are reasonable:

- [ ] At rest: All currents near 0A
- [ ] At 30% voltage: Currents between 5-20A (varies by motor)
- [ ] Under load: Currents increase proportionally
- [ ] Stall detection: Current spikes > 40A indicates jam

## Parameter Tuning Test

- [ ] Adjust Shooter Target RPM to 2000 on dashboard
- [ ] Press RB to spin up
- [ ] Observe Shooter RPM converge toward 2000
- [ ] Verify no recompile was needed
- [ ] Adjust again to 3000, verify new target
- [ ] Record what RPM values work best

## Safety Checks

- [ ] All voltage tests start at 30%
- [ ] Motors respond immediately to button release (coast or brake)
- [ ] No unexpected movement when sticks are neutral
- [ ] No motors spinning at startup
- [ ] Current limiting prevents runaway current

## Post-Test Data Recording

- [ ] Record working RPM values for each motor
- [ ] Record reasonable voltage percentages
- [ ] Note any mechanical issues encountered
- [ ] Document PID gains if tuned
- [ ] Record position tolerances that work well

## Ready to Proceed

- [ ] All above checks pass ✅
- [ ] System is stable and responsive
- [ ] Data logging capability verified
- [ ] Ready to move to competition commands

## Troubleshooting Notes

If any check fails, record here:

```
Issue: [Description]
Solution: [What was done to fix]
Result: [Verified working]

---

Issue: [Description]
Solution: [What was done to fix]
Result: [Verified working]
```

## Build and Deploy Commands

When ready to test:

```bash
# Build the project
./gradlew build

# Deploy to robot (while connected via USB)
./gradlew deploy

# Or with verbose output for debugging
./gradlew deploy --info
```

## Final Sign-Off

- [ ] All tests completed successfully
- [ ] System ready for operator training
- [ ] Data collection can begin
- [ ] Ready to migrate to competition code

**Date Completed:** ________________

**Operator Name:** ________________

**Notes:** 

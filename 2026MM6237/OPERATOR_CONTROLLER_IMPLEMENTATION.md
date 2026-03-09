# Operator Controller and SubsystemTuning Implementation Summary

## What Was Created

### 1. **OperatorController.java** 
Location: `src/main/java/frc/robot/controllers/OperatorController.java`

A comprehensive operator controller that provides safe, intuitive button mappings for testing and exercising all subsystems:

**Features:**
- Safe voltage control at 30% default for initial testing
- Bumper and trigger controls for Feeder and Shooter
- Face buttons and D-Pad controls for all other subsystems  
- Analog stick control for Hood servo positioning
- Deadband filtering to prevent unintended motion

**Subsystems Covered:**
- ✅ Feeder (forward/reverse)
- ✅ Shooter (forward/reverse)
- ✅ Intake (forward/stow/agitate/homing)
- ✅ Hood (analog position control)
- ✅ Hanger (extend/retract)
- ✅ Floor (feed forward)

**Button Mapping Reference:**
```
LB = Feeder Forward      |  RB = Shooter Forward
LT = Feeder Reverse      |  RT = Shooter Reverse
Y  = Intake Forward      |  X  = Intake Stow
B  = Intake Agitate      |  A  = Intake Home
Left Stick Y = Hood Position
DPad Up = Hanger Extend  |  DPad Down = Hanger Retract
DPad Left = Floor Feed   |  DPad Right = Floor Stop
```

### 2. **SubsystemTuning.java**
Location: `src/main/java/frc/robot/util/SubsystemTuning.java`

A centralized SmartDashboard integration class that provides:

**For Each Subsystem:**
- Real-time display of current motor RPM, currents, and positions
- Editable tuning parameters (gains, voltages, tolerances)
- Status indicators (homed, within tolerance, etc.)

**Organized Into Static Classes:**
- `FeederTuning` - Feed speed and current monitoring
- `ShooterTuning` - RPM feedback and PID gain tuning
- `IntakeTuning` - Pivot angle, roller speed, and homing parameters
- `HoodTuning` - Position limits and tolerance tuning
- `HangerTuning` - Extension, PID gains, and homing parameters
- `FloorTuning` - Feed speed and current monitoring

**Master Methods:**
- `initializeAllDashboards()` - Initialize all displays (call once at startup)
- `updateAllDashboards(...)` - Update all displays and read tuning values (call periodically)

### 3. **Enhanced Subsystem Getter Methods**

Added public getter methods to each subsystem for real-time data access:

**Feeder:**
- `getCurrentRPM()`, `getStatorCurrent()`, `getSupplyCurrent()`

**Shooter:**
- `getLeftMotorRPM()`, `getMiddleMotorRPM()`, `getRightMotorRPM()`
- `getLeftStatorCurrent()`, `getMiddleStatorCurrent()`, `getRightStatorCurrent()`

**Intake:**
- `getPivotAngleDegrees()`, `getRollerRPM()`
- `getPivotStatorCurrent()`, `getRollerStatorCurrent()`
- `isHomed()`

**Hood:**
- `getCurrentPosition()`, `getTargetPosition()`

**Hanger:**
- `getCurrentExtensionInches()`, `getSupplyCurrent()`

**Floor:**
- `getCurrentRPM()`, `getStatorCurrent()`, `getSupplyCurrent()`

### 4. **RobotContainer Updates**

**Changes Made:**
- Added import for `SubsystemTuning`
- Initialize SubsystemTuning in constructor: `SubsystemTuning.initializeAllDashboards()`
- Added public getter methods for all subsystems

### 5. **Robot.java Updates**

**Changes Made:**
- Update `robotPeriodic()` to call `SubsystemTuning.updateAllDashboards()` with all subsystem instances

### 6. **Documentation Files**

**SUBSYSTEM_TUNING_GUIDE.md**
Comprehensive guide covering:
- Setup instructions
- SmartDashboard parameter reference for each subsystem
- Usage examples in commands
- Safety notes and best practices

## SmartDashboard Organization

All tuning parameters are organized hierarchically:

```
Feeder/
  - Current RPM (RO)
  - Stator Current (RO)
  - Supply Current (RO)
  - Target RPM (Tunable)
  - Test Voltage % (Tunable)

Shooter/
  - Left/Middle/Right Motor RPM (RO)
  - Left/Middle/Right Stator Current (RO)
  - Target RPM (Tunable)
  - Test Voltage % (Tunable)
  - Left KP, KI, KD (Tunable)

Intake/
  - Pivot Angle (RO)
  - Roller RPM (RO)
  - Pivot/Roller Stator Current (RO)
  - Is Homed (RO)
  - Intake Speed % (Tunable)
  - Test Voltage % (Tunable)
  - Pivot KP, KI, KD (Tunable)
  - Homing Voltage % (Tunable)

Hood/
  - Current Position (RO)
  - Target Position (RO)
  - Within Tolerance (RO)
  - Min/Max Position (Tunable)
  - Position Tolerance (Tunable)
  - Test Position (Tunable)

Hanger/
  - Extension (RO)
  - Supply Current (RO)
  - Is Homed (RO)
  - Test Voltage % (Tunable)
  - KP, KI, KD (Tunable)
  - Homing Voltage % (Tunable)
  - Homing Current Threshold (Tunable)

Floor/
  - Current RPM (RO)
  - Stator Current (RO)
  - Supply Current (RO)
  - Feed Speed % (Tunable)
  - Test Voltage % (Tunable)
```

Legend: RO = Read-Only Display

## Key Benefits

1. **Safe Testing** - 30% voltage default prevents mechanical damage during initial testing
2. **No Recompilation** - Adjust parameters on the fly via SmartDashboard
3. **Real-Time Feedback** - Monitor motor performance during testing
4. **Systematic Approach** - All subsystems organized in one place
5. **Current Monitoring** - Detect mechanical binding or issues from current draw
6. **PID Tuning** - Adjust gains interactively for motion control
7. **Operator-Friendly** - Intuitive button mapping for basic operations

## Usage Workflow

### Step 1: Initialize (done in RobotContainer)
```java
SubsystemTuning.initializeAllDashboards();
```

### Step 2: Update Periodically (done in Robot.robotPeriodic)
```java
SubsystemTuning.updateAllDashboards(
    robotContainer.getFeeder(),
    robotContainer.getShooter(),
    robotContainer.getIntake(),
    robotContainer.getHood(),
    robotContainer.getHanger(),
    robotContainer.getFloor()
);
```

### Step 3: Test with Operator Controller
- Connect operator to Port 1
- Use mapped buttons to exercise subsystems
- Observe real-time data on SmartDashboard

### Step 4: Tune Parameters
- Adjust values in SmartDashboard
- Record values that work well
- Transfer back to Constants.java when finalized

## Next Steps

1. **Test the Operator Controller**
   - Verify all button mappings work as intended
   - Check for any mechanical issues during initial low-voltage testing

2. **Monitor SmartDashboard**
   - Watch current draw for signs of mechanical binding
   - Verify motor RPMs and positions update correctly

3. **Fine-Tune Parameters**
   - Adjust motor speeds as needed
   - Tune PID gains for smooth motion
   - Record optimized values to Constants.java

4. **Safety Verification**
   - Test current limiting is working
   - Verify mechanical limits and stops
   - Confirm deadbands prevent unintended motion

## Safety Reminders

⚠️ **Always:**
- Start with low voltages (30% is default for testing)
- Verify mechanical clearances before testing
- Monitor current draw for signs of issues
- Use deadbands to filter stick noise
- Have an operator ready to disable if needed

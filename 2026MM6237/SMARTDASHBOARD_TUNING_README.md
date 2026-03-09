# SmartDashboard Tuning and Operator Controller - Complete Implementation

## Overview

This implementation provides a comprehensive system for testing, monitoring, and tuning all subsystems through an intuitive Operator Controller and real-time SmartDashboard interface. This is perfect for:

- ✅ Safe initial subsystem testing at 30% voltage
- ✅ Real-time motor feedback (RPM, current, position)
- ✅ PID gain tuning without recompilation
- ✅ Current monitoring for mechanical diagnostics
- ✅ Systematic data collection for optimization

## Files Created and Modified

### New Files Created:

1. **`src/main/java/frc/robot/controllers/OperatorController.java`**
   - Button mappings for all subsystems
   - Safe voltage defaults for testing
   - Deadband filtering and analog control

2. **`src/main/java/frc/robot/util/SubsystemTuning.java`**
   - Centralized SmartDashboard display management
   - Real-time sensor data display
   - Tunable parameter interface
   - One-line initialization and update methods

3. **`src/main/java/frc/robot/commands/TuningExampleCommands.java`**
   - Example command implementations using tuning values
   - Multi-subsystem command examples
   - Validation and diagnostic utilities

4. **`SUBSYSTEM_TUNING_GUIDE.md`**
   - Complete parameter reference
   - Setup instructions
   - Usage examples
   - Safety guidelines

5. **`OPERATOR_CONTROLLER_IMPLEMENTATION.md`**
   - Implementation summary
   - Button mapping reference
   - Workflow guidance
   - Next steps

### Modified Files:

1. **`src/main/java/frc/robot/subsystems/Feeder.java`**
   - Added: `getCurrentRPM()`, `getStatorCurrent()`, `getSupplyCurrent()`

2. **`src/main/java/frc/robot/subsystems/Shooter.java`**
   - Added: `getLeftMotorRPM()`, `getMiddleMotorRPM()`, `getRightMotorRPM()`
   - Added: `getLeftStatorCurrent()`, `getMiddleStatorCurrent()`, `getRightStatorCurrent()`

3. **`src/main/java/frc/robot/subsystems/Intake.java`**
   - Added: `getPivotAngleDegrees()`, `getRollerRPM()`
   - Added: `getPivotStatorCurrent()`, `getRollerStatorCurrent()`, `isHomed()`

4. **`src/main/java/frc/robot/subsystems/Hood.java`**
   - Added: `getCurrentPosition()`, `getTargetPosition()`

5. **`src/main/java/frc/robot/subsystems/Hanger.java`**
   - Added: `getCurrentExtensionInches()`, `getSupplyCurrent()`

6. **`src/main/java/frc/robot/subsystems/Floor.java`**
   - Added: `getCurrentRPM()`, `getStatorCurrent()`, `getSupplyCurrent()`

7. **`src/main/java/frc/robot/RobotContainer.java`**
   - Added: Import for `SubsystemTuning`
   - Added: `SubsystemTuning.initializeAllDashboards()` in constructor
   - Added: Public getter methods for all subsystems

8. **`src/main/java/frc/robot/Robot.java`**
   - Added: `SubsystemTuning.updateAllDashboards()` in `robotPeriodic()`

## Quick Start

### 1. Build and Deploy
```bash
./gradlew build
./gradlew deploy
```

### 2. Connect Operator Controller
- Plug Xbox controller into Port 1 (the second port)
- Port 0 remains for driver

### 3. Launch SmartDashboard
- Open SmartDashboard
- Look for tuning tabs organized by subsystem name

### 4. Test Subsystems
- Use button mappings to exercise each subsystem
- Monitor real-time values on SmartDashboard
- Adjust tuning parameters as needed

## Operator Controller Button Map

| Function | Button | Action |
|----------|--------|--------|
| **Feeder** | LB | Spin Forward (30% voltage) |
| **Feeder** | LT | Spin Reverse (30% voltage) |
| **Shooter** | RB | Spin Forward (30% voltage) |
| **Shooter** | RT | Spin Reverse (30% voltage) |
| **Intake** | Y | Forward (Rollers + Pivot) |
| **Intake** | X | Stow (Stop + Move Pivot) |
| **Intake** | B | Agitate Sequence |
| **Intake** | A | Home Sequence |
| **Hood** | Left Stick Y | Analog Position (±5% per frame) |
| **Hanger** | DPad ↑ | Extend (30% voltage) |
| **Hanger** | DPad ↓ | Retract (30% voltage) |
| **Floor** | DPad ← | Feed Forward |
| **Floor** | DPad → | Stop |

## SmartDashboard Organization

Navigate to the "SmartDashboard" tab and look for subsystem-specific tabs:

- **Feeder/** - RPM, current, test parameters
- **Shooter/** - Left/Middle/Right motor RPM, currents, PID gains
- **Intake/** - Angle, RPM, currents, homing status
- **Hood/** - Position, limits, tolerance
- **Hanger/** - Extension, current, homing status
- **Floor/** - RPM, current, feed speed

## Real-Time Data Available

All values update at 50 Hz (every 20ms):

### Motor Data
- RPM/velocity for all motors
- Stator current (motor load)
- Supply current (battery load)

### Position Data
- Hood servo position (0.0-1.0)
- Intake pivot angle (degrees)
- Hanger extension (inches)

### Status Indicators
- Is homed (boolean)
- Within tolerance (boolean)
- Current command name

## Tuning Workflow

### Phase 1: Initial Testing
1. Set all test voltages to 30%
2. Manually exercise each subsystem
3. Monitor for mechanical issues
4. Verify current limiting works

### Phase 2: Characterization
1. Gradually increase voltage to find stall currents
2. Record motor behavior at different loads
3. Verify mechanical limits and stops
4. Check for grinding or binding

### Phase 3: Tuning
1. Adjust RPM setpoints based on mechanical limits
2. Fine-tune PID gains for smooth motion
3. Adjust tolerances and thresholds
4. Test in realistic sequences

### Phase 4: Validation
1. Record working parameters from dashboard
2. Copy values to Constants.java
3. Verify behavior persists after restart
4. Test in realistic game scenarios

## Example: Adjusting Shooter RPM

1. **On SmartDashboard:**
   - Find `Shooter/Target RPM` widget
   - Type desired RPM (e.g., 3000)

2. **In Code:**
   - Command automatically uses new value
   - No rebuild/redeploy needed

3. **Test:**
   - Press RB to spin up
   - Watch `Shooter/Left|Middle|Right Motor RPM` converge to target

4. **Record:**
   - Note the working RPM value
   - Update `Constants.Shooter.kFeedRPM` when ready

## Safety Features

✅ **Built-In Protections:**
- 30% voltage default prevents mechanical damage
- Deadbands filter stick noise
- Current limiting prevents stalls
- Position clamping prevents mechanical damage
- Clear command names for operator awareness

⚠️ **Operator Responsibilities:**
- Start with low voltages
- Monitor currents for binding
- Watch for unusual sounds
- Stop immediately if issues develop
- Keep hands clear during testing

## Example Commands

See `TuningExampleCommands.java` for implementations of:

- Feeder feed using dashboard RPM
- Shooter spin-up using dashboard RPM
- Intake sequence with dashboard tuning
- Hood positioning with tolerance
- Hanger homing with dashboard voltage
- Multi-subsystem prepared-to-shoot command
- Tuning validation utilities

Copy these examples to your command classes and adapt as needed.

## Troubleshooting

### SmartDashboard shows no tuning tabs
- Verify `SubsystemTuning.initializeAllDashboards()` was called
- Check Robot.robotPeriodic() includes the update call
- Rebuild and redeploy

### Values not updating on dashboard
- Check `robotPeriodic()` is being called (look for Scheduler output)
- Verify getter methods are returning correct data
- Check network connectivity to the robot

### Subsystem not responding to buttons
- Verify operator controller on Port 1
- Check button mappings in `OperatorController.java`
- Verify subsystem is created and initialized
- Check for higher-priority command conflicts

### Motors spinning unexpectedly
- Verify deadband value (should be ~0.1)
- Check stick center position
- Test with all sticks in neutral position
- Reduce voltage percentage further for testing

## Next Steps

1. ✅ **Test all button mappings** - Verify each subsystem responds
2. ✅ **Monitor SmartDashboard** - Check all values update
3. ✅ **Characterize subsystems** - Find RPM/force limits
4. ✅ **Tune PID gains** - Optimize motion smoothness
5. ✅ **Record working values** - Document in Constants.java
6. ✅ **Build competition commands** - Use tuning values as base
7. ✅ **Validate in game scenarios** - Test realistic sequences

## References

- See `SUBSYSTEM_TUNING_GUIDE.md` for complete parameter reference
- See `OPERATOR_CONTROLLER_IMPLEMENTATION.md` for detailed implementation notes
- See `TuningExampleCommands.java` for code examples

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review SmartDashboard tab names and values
3. Verify all modified files compiled without errors
4. Check RoboRIO logs for exceptions

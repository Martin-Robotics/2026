# SmartDashboard Layout Guide

## Dashboard Organization

When you launch SmartDashboard with the robot, you'll see subsystem data organized as follows:

## Main Dashboard Tabs

### SmartDashboard Tab (Default)

```
┌─────────────────────────────────────────────────────┐
│ SmartDashboard                                      │
├─────────────────────────────────────────────────────┤
│                                                     │
│  Field              [Field 2D Display]             │
│  Auto Mode          [DefaultPath ▼]                │
│                                                     │
│  Subsystem Tuning                                  │
│  Status: Ready                                      │
│                                                     │
│  ─────────────────────────────────────────────     │
│  Feeder/Current RPM           [####] 0.0           │
│  Feeder/Stator Current (A)    [####] 0.0           │
│  Feeder/Supply Current (A)    [####] 0.0           │
│                                                     │
│  ... (rest of read-only displays)                 │
│                                                     │
└─────────────────────────────────────────────────────┘
```

## Expandable Subsystem Sections

Click on subsystem names to expand/collapse:

### Feeder Section
```
▼ Feeder/
  ├─ Current RPM (RO)              [Display: 0.0]
  ├─ Stator Current (A) (RO)       [Display: 0.0]
  ├─ Supply Current (A) (RO)       [Display: 0.0]
  ├─ Target RPM (Tunable)          [Slider: 5000]
  └─ Test Voltage % (Tunable)      [Slider: 0.3]
```

### Shooter Section
```
▼ Shooter/
  ├─ Left Motor RPM (RO)           [Display: 0.0]
  ├─ Middle Motor RPM (RO)         [Display: 0.0]
  ├─ Right Motor RPM (RO)          [Display: 0.0]
  ├─ Left Stator Current (A) (RO)  [Display: 0.0]
  ├─ Middle Stator Current (A) (RO)[Display: 0.0]
  ├─ Right Stator Current (A) (RO) [Display: 0.0]
  ├─ Target RPM (Tunable)          [Slider: 0.0]
  ├─ Test Voltage % (Tunable)      [Slider: 0.3]
  ├─ Left KP (Tunable)             [Slider: 0.5]
  ├─ KI (Tunable)                  [Slider: 2.0]
  └─ KD (Tunable)                  [Slider: 0.0]
```

### Intake Section
```
▼ Intake/
  ├─ Pivot Angle (RO)              [Display: 110°]
  ├─ Roller RPM (RO)               [Display: 0.0]
  ├─ Pivot Stator Current (A) (RO) [Display: 0.0]
  ├─ Roller Stator Current (A) (RO)[Display: 0.0]
  ├─ Is Homed (RO)                 [Display: true]
  ├─ Intake Speed % (Tunable)      [Slider: 0.8]
  ├─ Test Voltage % (Tunable)      [Slider: 0.3]
  ├─ Pivot KP (Tunable)            [Slider: 300]
  ├─ Pivot KI (Tunable)            [Slider: 0]
  ├─ Pivot KD (Tunable)            [Slider: 0]
  └─ Homing Voltage % (Tunable)    [Slider: 0.1]
```

### Hood Section
```
▼ Hood/
  ├─ Current Position (RO)         [Display: 0.50]
  ├─ Target Position (RO)          [Display: 0.50]
  ├─ Within Tolerance (RO)         [Display: true]
  ├─ Min Position (Tunable)        [Slider: 0.01]
  ├─ Max Position (Tunable)        [Slider: 0.77]
  ├─ Position Tolerance (Tunable)  [Slider: 0.01]
  └─ Test Position (Tunable)       [Slider: 0.50]
```

### Hanger Section
```
▼ Hanger/
  ├─ Extension (inches) (RO)       [Display: 0.0]
  ├─ Supply Current (A) (RO)       [Display: 0.0]
  ├─ Is Homed (RO)                 [Display: false]
  ├─ Test Voltage % (Tunable)      [Slider: 0.3]
  ├─ KP (Tunable)                  [Slider: 10]
  ├─ KI (Tunable)                  [Slider: 0]
  ├─ KD (Tunable)                  [Slider: 0]
  ├─ Homing Voltage % (Tunable)    [Slider: -0.05]
  └─ Homing Current Threshold (A)  [Slider: 0.4]
```

### Floor Section
```
▼ Floor/
  ├─ Current RPM (RO)              [Display: 0.0]
  ├─ Stator Current (A) (RO)       [Display: 0.0]
  ├─ Supply Current (A) (RO)       [Display: 0.0]
  ├─ Feed Speed % (Tunable)        [Slider: 0.83]
  └─ Test Voltage % (Tunable)      [Slider: 0.3]
```

## How to Use Sliders

### To Change a Value:

1. **Hover over the slider**
   ```
   ╔═══════════════════════════════════════════════════════╗
   ║ KP         ├───────●───────┤                    300    ║
   │            └─ Drag this indicator to adjust value
   ╚═══════════════════════════════════════════════════════╝
   ```

2. **Click and drag left/right**
   - Drag left: Decrease value
   - Drag right: Increase value
   - Current value shown on right

3. **Or type directly**
   - Click on the value field (right side)
   - Clear current value
   - Type new value
   - Press Enter

4. **Release to apply**
   - Value is immediately used by code
   - No rebuild/redeploy needed

## Reading Displays

### Motor RPM Displays
```
Shooter/Left Motor RPM    [Display showing: 2850 ]
                                     ↑
                          Real-time value in RPM
```

### Current Displays
```
Feeder/Stator Current     [Display showing: 12.3 A]
                                     ↑
                          Motor load in Amps
```

### Position Displays
```
Hood/Current Position     [Display showing: 0.532]
                                     ↑
                          Servo position (0.0-1.0)

Hanger/Extension          [Display showing: 3.25 inches]
                                     ↑
                          Extension measurement
```

### Boolean Displays
```
Intake/Is Homed           [Display showing: true]
Hood/Within Tolerance     [Display showing: false]
                                     ↑
                          On/Off or True/False
```

## Typical Parameter Ranges

### Voltages (0.0 - 1.0 scale)
- Safe testing: 0.2 - 0.4 (20-40%)
- Normal operation: 0.5 - 1.0 (50-100%)
- Emergency: > 1.0 usually clamped to 1.0

### Motor Currents (Amps)
- At rest: 0 - 1 A
- Light load: 5 - 15 A
- Medium load: 15 - 30 A
- Heavy load: 30 - 50 A
- Stall (error): > 50 A

### RPM Values
- Feeder: 0 - 6000 RPM
- Shooter: 0 - 6000 RPM per motor
- Intake rollers: 0 - 6000 RPM

### Positions
- Hood servo: 0.0 (fully closed) to 1.0 (fully open)
- Intake angle: -4° to 110° (or configured range)
- Hanger extension: 0" to ~6"

## Monitoring During Operation

### Watch These Values:

1. **Current Draw** - Indicates load/binding
   - Sudden spike = something stuck
   - Gradual increase = normal under load
   - Stays at 0 = motor not moving (check!)

2. **RPM** - Verifies motion
   - Should increase when voltage applied
   - Should decrease when released
   - Lag indicates load or mechanical issue

3. **Position** - Confirms location
   - Should reach target position
   - Should stay within tolerance
   - Should change smoothly, not jump

4. **Status Flags** - Quick diagnostics
   - "Is Homed" = position reference established
   - "Within Tolerance" = close enough to target

## Common Adjustments

### To Speed Up a Motor
1. Increase the Test Voltage %
2. Observe RPM increase
3. Record working value

### To Tune PID Gains
1. Increase KP slowly (1-10 increments)
2. Watch for oscillation in RPM/position
3. If oscillating, reduce KP
4. If still not responsive, increase KI
5. Use KD to dampen oscillation

### To Find Safe Voltage
1. Start at 0.1 (10%)
2. Increase by 0.05 increments
3. Watch for smooth motion
4. Note when motor starts moving
5. Use that voltage +10% for testing

## Dashboard Persistence

- **Values saved between reboots** ✓
- **Slider positions remembered** ✓
- **Read-only displays reset** ✓ (always live data)

To reset everything:
1. Fully restart SmartDashboard
2. Reconnect to robot
3. All values revert to defaults

## Troubleshooting Display Issues

| Problem | Solution |
|---------|----------|
| No subsystem tabs appear | Verify SubsystemTuning.initialize() was called |
| Values not updating | Check Robot.robotPeriodic() has update call |
| Slider moves but value doesn't | Subsystem getter may not be reading correctly |
| Display shows "N/A" | Motor/sensor not connected or reporting error |
| Old values stick around | Restart SmartDashboard after code changes |

## Tips for Effective Monitoring

1. **Arrange key displays prominently** - Drag display positions
2. **Expand only active sections** - Reduce visual clutter
3. **Monitor one subsystem at a time** - Don't overwhelm yourself
4. **Take notes on good values** - Pen and paper, or screenshot
5. **Compare before/after** - Screenshot before tuning, after tuning
6. **Check all three motors** - Shooter motors should match RPM

## Example: Real-Time Tuning

Scenario: Shooter spin-up feels slow

**Step 1:** Observe
```
Shooter/Left Motor RPM      [Display: 0 → 1200]
Shooter/Middle Motor RPM    [Display: 0 → 950]
Shooter/Right Motor RPM     [Display: 0 → 1100]
                            Uneven, middle is slower
```

**Step 2:** Adjust
```
Shooter/Left KP             [Slider: 0.5 → 0.7]
Watch for improvement
```

**Step 3:** Test
```
Press RB, watch RPM rise again
Now Middle Motor shows: 0 → 1150 (better!)
```

**Step 4:** Record
```
Note: Left KP = 0.7 works better than 0.5
```

---

Legend:
- **(RO)** = Read-Only Display (cannot be changed)
- **(Tunable)** = Slider/Input (can be changed real-time)
- **[Display: X]** = Example output value
- **[Slider: X]** = Adjustable parameter

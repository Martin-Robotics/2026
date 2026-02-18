# Autonomous Commands - PathPlanner Integration Guide

## 📁 Files Created
Located in: `src/main/java/frc/robot/commands/auto/`

1. **PrepareForIntake.java** - Positions intake arm for collecting notes
2. **RunIntake.java** - Spins intake rollers to collect notes
3. **StopIntake.java** - Stops intake and retracts arm
4. **PrepareToFire.java** - Spins up shooter based on distance
5. **Fire.java** - Feeds note through shooter for a set duration
6. **PrepareToClimb.java** - Prepares hanger for climbing
7. **Climb.java** - Engages climb motors to hang
8. **README.md** - Comprehensive documentation

## 🚀 Quick Start with PathPlanner

### Step 1: Commands are Pre-Registered ✅
All commands are automatically registered as NamedCommands in `RobotContainer.java`. No manual setup required!

### Step 2: Use in PathPlanner `.auto` Files
Reference commands by their registered names in JSON:

```json
{
  "type": "named",
  "data": {
    "name": "PrepareForIntake"
  }
}
```

### Step 3: Build Auto Sequences
Compose commands in PathPlanner's visual editor:
- Add path segments
- Add command events (select from registered commands)
- Add wait events for timing
- Add parallel markers for simultaneous execution

## 📋 Available Commands

These command names are automatically available in PathPlanner:

### Intake Commands
- **PrepareForIntake** - Moves intake arm to position
- **RunIntake** - Spins intake rollers (runs until interrupted)
- **StopIntake** - Stops rollers and retracts arm

### Shooter Commands
- **PrepareToFire** - Spins up shooter based on distance
- **Fire** - Feeds note through shooter (duration-based)

### Climb Commands
- **PrepareToClimb** - Extends hanger to intermediate position
- **Climb** - Completes climb to hung position

## 🎯 Example Auto Sequences

### Simple Intake Sequence
In PathPlanner visual editor:
1. Path: "Example Path"
2. Command: PrepareForIntake
3. Command: RunIntake
4. Wait: 2.0 seconds
5. Command: StopIntake

### Collect and Shoot Sequence
1. Path: "Collect Path"
2. Command: PrepareForIntake
3. Command: RunIntake
4. Wait: 2.0 seconds
5. Command: StopIntake
6. Path: "Shoot Path"
7. Command: PrepareToFire
8. Command: Fire

### Climb Sequence
1. Path: "Climb Path"
2. Command: PrepareToClimb
3. Command: Climb

## ⚙️ Key Configuration Points

### Fire Duration
Edit `Fire.java` line 20:
```java
private static final double FIRE_DURATION_SECONDS = 0.5;
```

### Distance-based RPM (PrepareToFire)
Edit the `calculateShooterRPM()` method to adjust shooter RPM based on distance from hub.

### Distance-based Feeder Speed (Fire)
Edit the `calculateFeederSpeed()` method to adjust feeder output based on distance from hub.

### Hub AprilTag ID
Both PrepareToFire and Fire use tag ID 4. Change if your field uses different IDs:
```java
private static final int HUB_TAG_ID = 4;
```

## 📊 Registration Summary

All commands are registered in `RobotContainer.java`:

```java
private void registerNamedCommands() {
    // Intake commands
    NamedCommands.registerCommand("PrepareForIntake", new PrepareForIntake(intake));
    NamedCommands.registerCommand("RunIntake", new RunIntake(intake));
    NamedCommands.registerCommand("StopIntake", new StopIntake(intake));
    
    // Shooter commands
    NamedCommands.registerCommand("PrepareToFire", new PrepareToFire(shooter, limelight));
    NamedCommands.registerCommand("Fire", new Fire(feeder, shooter, limelight));
    
    // Climb commands
    NamedCommands.registerCommand("PrepareToClimb", new PrepareToClimb(hanger));
    NamedCommands.registerCommand("Climb", new Climb(hanger));
}
```

## ✅ All Commands Verified
- ✓ No compilation errors
- ✓ Proper subsystem requirements declared
- ✓ Automatically registered with PathPlanner
- ✓ Distance-based tuning functions included
- ✓ Ready to use in PathPlanner autos

## 📖 Documentation
See `README.md` in the auto commands folder for detailed documentation of each command.

## 🔗 Next Steps
1. Open your PathPlanner auto file editor
2. Create or edit `.auto` files using the registered command names
3. Test sequences with individual commands first
4. Combine into complete autonomous routines
5. Tune distance-based calculations based on actual robot performance
6. Test on actual field setup

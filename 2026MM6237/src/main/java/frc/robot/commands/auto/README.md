# Autonomous Commands Summary

This document provides an overview of the autonomous commands created for the 2026 robot, which are designed to work with **PathPlanner's NamedCommands system**.

## Command Directory
All commands are located in: `src/main/java/frc/robot/commands/auto/`

## NamedCommands Registration

All commands are automatically registered in `RobotContainer.java` via the `registerNamedCommands()` method:

| Command Name | Class | Subsystems |
|--------------|-------|-----------|
| `PrepareForIntake` | PrepareForIntake | Intake |
| `RunIntake` | RunIntake | Intake |
| `StopIntake` | StopIntake | Intake |
| `PrepareToFire` | PrepareToFire | Shooter, Limelight |
| `Fire` | Fire | Feeder, Shooter, Limelight |
| `PrepareToClimb` | PrepareToClimb | Hanger |
| `Climb` | Climb | Hanger |

These names can be used directly in PathPlanner `.auto` files as NamedCommand events.

## Commands Overview

### 1. **PrepareForIntake**
- **Package**: `frc.robot.commands.auto`
- **Class**: `PrepareForIntake extends Command`
- **Constructor**: `PrepareForIntake(Intake intake)`
- **Purpose**: Moves the intake arm to the INTAKE position to prepare for collecting notes
- **Behavior**:
  - Sets intake arm to `Position.INTAKE`
  - Waits for arm to reach target position (within 5° tolerance)
  - Completes when arm is positioned
- **Subsystem Requirements**: Intake
- **Usage**:
  ```java
  new PrepareForIntake(intakeSubsystem)
  ```

### 2. **RunIntake**
- **Package**: `frc.robot.commands.auto`
- **Class**: `RunIntake extends Command`
- **Constructor**: `RunIntake(Intake intake)`
- **Purpose**: Spins up the intake rollers to collect notes
- **Behavior**:
  - Sets intake rollers to `Speed.INTAKE`
  - Rollers continue spinning until interrupted
  - Command runs indefinitely (must be sequenced with other commands)
  - Rollers remain spinning when command ends
- **Subsystem Requirements**: Intake
- **Usage**:
  ```java
  new RunIntake(intakeSubsystem).withTimeout(3.0) // Run for 3 seconds
  ```

### 3. **StopIntake**
- **Package**: `frc.robot.commands.auto`
- **Class**: `StopIntake extends Command`
- **Constructor**: `StopIntake(Intake intake)`
- **Purpose**: Stops intake rollers and retracts the arm to stowed position
- **Behavior**:
  - Stops rollers immediately (`Speed.STOP`)
  - Moves arm to `Position.STOWED`
  - Waits for arm to reach stowed position (within 5° tolerance)
  - Completes when arm is positioned
- **Subsystem Requirements**: Intake
- **Usage**:
  ```java
  new StopIntake(intakeSubsystem)
  ```

### 4. **PrepareToFire**
- **Package**: `frc.robot.commands.auto`
- **Class**: `PrepareToFire extends Command`
- **Constructor**: `PrepareToFire(Shooter shooter, LimelightSubsystem6237 limelight)`
- **Purpose**: Spins up shooter rollers at RPM determined by distance to hub
- **Behavior**:
  - Measures distance to hub using Limelight (AprilTag ID 4)
  - Calculates shooter RPM based on distance (2000-5500 RPM range)
  - Uses default RPM (5000) if hub not visible
  - Spins rollers to target velocity
  - Completes when rollers reach target speed
  - Rollers remain spinning when command ends
- **Subsystem Requirements**: Shooter, LimelightSubsystem6237 (optional target detection)
- **Distance-to-RPM Mapping** (editable):
  - 1.0m → 2000 RPM
  - 8.0m → 5500 RPM
  - Linear interpolation between ranges
- **Usage**:
  ```java
  new PrepareToFire(shooterSubsystem, limelightSubsystem)
  ```

### 5. **Fire**
- **Package**: `frc.robot.commands.auto`
- **Class**: `Fire extends Command`
- **Constructor**: `Fire(Feeder feeder, Shooter shooter, LimelightSubsystem6237 limelight)`
- **Purpose**: Feeds the note through the shooter for a fixed duration
- **Behavior**:
  - Runs for `FIRE_DURATION_SECONDS` (0.5 seconds, tunable)
  - Measures distance to hub using Limelight (AprilTag ID 4)
  - Calculates feeder speed based on distance (0.6-1.0 output range)
  - Feeder speed increases with distance
  - After duration elapses, stops feeder and shooter rollers
- **Subsystem Requirements**: Feeder, Shooter, LimelightSubsystem6237 (optional distance detection)
- **Configuration Constants**:
  - `FIRE_DURATION_SECONDS = 0.5` (adjust for your robot)
  - Distance range: 1.0m - 8.0m
  - Feeder output range: 0.6 - 1.0
- **Usage**:
  ```java
  new Fire(feederSubsystem, shooterSubsystem, limelightSubsystem)
  ```

### 6. **PrepareToClimb**
- **Package**: `frc.robot.commands.auto`
- **Class**: `PrepareToClimb extends Command`
- **Constructor**: `PrepareToClimb(Hanger hanger)`
- **Purpose**: Prepares the robot for climbing by extending hanger to intermediate position
- **Behavior**:
  - Moves hanger to `Position.EXTEND_HOPPER`
  - Waits for hanger to reach target extension (within 1 inch tolerance)
  - Completes when hanger is positioned
- **Subsystem Requirements**: Hanger
- **Usage**:
  ```java
  new PrepareToClimb(hangerSubsystem)
  ```

### 7. **Climb**
- **Package**: `frc.robot.commands.auto`
- **Class**: `Climb extends Command`
- **Constructor**: `Climb(Hanger hanger)`
- **Purpose**: Engages climb motors to climb and hang on the tower
- **Behavior**:
  - Moves to `Position.HANGING` first
  - Waits for that position to be reached
  - Then moves to `Position.HUNG`
  - Completes when fully hung (within 1 inch tolerance)
  - Maintains hung position when command ends
- **Subsystem Requirements**: Hanger
- **Usage**:
  ```java
  new Climb(hangerSubsystem)
  ```

## Example Autonomous Routines

### Using Commands in PathPlanner

In your `.auto` files, you can now reference these commands by their registered names. For example:

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "path",
          "data": {
            "pathName": "Example Path"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "PrepareForIntake"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "RunIntake"
          }
        },
        {
          "type": "wait",
          "data": {
            "waitTime": 2.0
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        }
      ]
    }
  },
  "resetOdom": true,
  "folder": null,
  "choreoAuto": false
}
```

### Collect and Shoot Example

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "named",
          "data": {
            "name": "PrepareForIntake"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "RunIntake"
          }
        },
        {
          "type": "wait",
          "data": {
            "waitTime": 2.0
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "PrepareToFire"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "Fire"
          }
        }
      ]
    }
  },
  "resetOdom": true,
  "folder": null,
  "choreoAuto": false
}
```

### Climb Example

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "named",
          "data": {
            "name": "PrepareToClimb"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "Climb"
          }
        }
      ]
    }
  },
  "resetOdom": true,
  "folder": null,
  "choreoAuto": false
}
```

## Integration Notes

### How NamedCommands Registration Works

1. When `RobotContainer` is instantiated, `registerNamedCommands()` is called in the constructor
2. Each command is registered with a unique name that PathPlanner can reference
3. PathPlanner `.auto` files use `"type": "named"` entries to invoke these commands
4. The commands execute sequentially or in parallel as defined in your auto file

### Key Integration Points

All commands follow WPILib's Command framework and are automatically registered via:

```java
// In RobotContainer.registerNamedCommands():
NamedCommands.registerCommand("PrepareForIntake", new PrepareForIntake(intake));
NamedCommands.registerCommand("RunIntake", new RunIntake(intake));
// ... etc for all 7 commands
```

**No additional code needed** - just reference the command names in PathPlanner!

## Tuning Constants

### Distance-based RPM Calculation (PrepareToFire)
Edit the `calculateShooterRPM()` method in `PrepareToFire.java`:
- `minDistance`: Closest reasonable shooting distance (meters)
- `maxDistance`: Farthest reasonable shooting distance (meters)
- `minRPM`: Shooter RPM at minimum distance
- `maxRPM`: Shooter RPM at maximum distance

### Feeder Speed Calculation (Fire)
Edit the `calculateFeederSpeed()` method in `Fire.java`:
- `minDistance`: Closest reasonable shooting distance (meters)
- `maxDistance`: Farthest reasonable shooting distance (meters)
- `minPercentOutput`: Feeder speed (0-1) at minimum distance
- `maxPercentOutput`: Feeder speed (0-1) at maximum distance

### Fire Duration (Fire)
Change `FIRE_DURATION_SECONDS` constant in `Fire.java` (default: 0.5 seconds)

### Position Tolerances
- Intake arm tolerance: 5 degrees (in PrepareForIntake and StopIntake)
- Hanger extension tolerance: 1 inch (in PrepareToClimb and Climb)

## Important Considerations

1. **Subsystem Requirements**: Each command specifies which subsystems it requires. Parallel commands cannot use the same subsystem.

2. **RunIntake Behavior**: This command runs indefinitely. Always use `.withTimeout()` or sequence it with other commands.

3. **Limelight Integration**: PrepareToFire and Fire use Limelight for distance measurement. They include fallback default values if the hub is not visible.

4. **Distance Calculations**: The distance-based RPM and feeder speed calculations use linear interpolation. These values should be tuned through testing on your specific robot.

5. **Roller Persistence**: PrepareToFire and RunIntake leave their motors running when the command ends. This is intentional for sequencing with Fire, which expects the shooter to already be spinning.

6. **AprilTag ID**: Both PrepareToFire and Fire assume the hub AprilTag has ID 4. Verify this and adjust the `HUB_TAG_ID` constant if needed.

# PathPlanner NamedCommands Usage Examples

This file contains example `.auto` file JSON structures showing how to use the registered autonomous commands in PathPlanner.

## Command Registration Reference

All commands are registered with these exact names:
- `PrepareForIntake`
- `RunIntake`
- `StopIntake`
- `PrepareToFire`
- `Fire`
- `PrepareToClimb`
- `Climb`

Use these names exactly as shown in your PathPlanner `.auto` files.

## Basic Command Template

```json
{
  "type": "named",
  "data": {
    "name": "CommandNameHere"
  }
}
```

## Complete Auto Examples

### Example 1: Simple Path with Intake

Collects a note on a simple straight path.

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
            "pathName": "SimpleIntakePath"
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

### Example 2: Collect and Shoot Auto

Drives to collect a note, then to the shooting position.

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
            "pathName": "CollectPath"
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
        },
        {
          "type": "path",
          "data": {
            "pathName": "ShootPath"
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

### Example 3: Multi-Note Collection

Collects multiple notes in sequence.

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
            "pathName": "FirstNotePath"
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
            "waitTime": 1.5
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        },
        {
          "type": "path",
          "data": {
            "pathName": "SecondNotePath"
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
            "waitTime": 1.5
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

### Example 4: Endgame Climb

Prepares and completes the climbing sequence.

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
            "pathName": "ClimbPath"
          }
        },
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

### Example 5: Full Score + Climb Auto

Complete autonomous routine for maximum points.

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
            "pathName": "CollectPath"
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
        },
        {
          "type": "path",
          "data": {
            "pathName": "ShootPath"
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
        },
        {
          "type": "path",
          "data": {
            "pathName": "ClimbPath"
          }
        },
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

### Example 6: Parallel Execution (Shooter Spin-up)

Starts shooter spin-up while following a path.

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "parallel",
          "data": {
            "commands": [
              {
                "type": "path",
                "data": {
                  "pathName": "DriveToShoot"
                }
              },
              {
                "type": "named",
                "data": {
                  "name": "PrepareToFire"
                }
              }
            ]
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

## Common Patterns

### Wait Between Commands
Use wait events to add delays:
```json
{
  "type": "wait",
  "data": {
    "waitTime": 1.0
  }
}
```

### Parallel Execution
Run multiple commands simultaneously (until one completes):
```json
{
  "type": "parallel",
  "data": {
    "commands": [
      { /* command 1 */ },
      { /* command 2 */ }
    ]
  }
}
```

### Command Groups
Sequential command groups for logical organization:
```json
{
  "type": "sequential",
  "data": {
    "commands": [
      { /* command 1 */ },
      { /* command 2 */ }
    ]
  }
}
```

## Tips for Autonomous Design

1. **Intake Timing**: RunIntake runs indefinitely, so always pair with a wait time
2. **Shooter Spin-up**: PrepareToFire completes when shooter reaches speed (can be long)
3. **Distance-based Adjustments**: PrepareToFire and Fire automatically adjust based on distance
4. **Test Incrementally**: Create simple paths first, then add complexity
5. **Use Parallel Execution**: Spin up shooter while driving to save time

## Debugging

- If a command doesn't work, verify the exact name matches the registered command
- Check `RobotContainer.registerNamedCommands()` for the exact spelling
- Use SmartDashboard to verify command execution
- Test commands individually before adding to complex sequences

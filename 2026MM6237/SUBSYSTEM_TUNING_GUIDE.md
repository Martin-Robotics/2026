# Subsystem Tuning and SmartDashboard Integration Guide

## Overview

The `SubsystemTuning` class provides centralized SmartDashboard displays and tunable parameters for all subsystems. This allows operators to view real-time data and adjust tuning values without recompiling code.

## Setup

### 1. Initialize in RobotContainer or Robot.robotInit()

Add this to your `RobotContainer` constructor or `Robot.robotInit()`:

```java
SubsystemTuning.initializeAllDashboards();
```

### 2. Update Periodically in Robot.robotPeriodic()

Add this to your `Robot.robotPeriodic()` method:

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

## SmartDashboard Displays and Tuning Parameters

### Feeder

**Read-Only Displays:**
- `Feeder/Current RPM` - Current motor RPM
- `Feeder/Stator Current (A)` - Stator current draw
- `Feeder/Supply Current (A)` - Supply current draw

**Tunable Parameters:**
- `Feeder/Target RPM` - Target RPM for feed operation (default: 5000)
- `Feeder/Test Voltage %` - Voltage percentage for testing (default: 0.3 = 30%)

### Shooter

**Read-Only Displays:**
- `Shooter/Left Motor RPM` - Left motor current RPM
- `Shooter/Middle Motor RPM` - Middle motor current RPM
- `Shooter/Right Motor RPM` - Right motor current RPM
- `Shooter/Left Stator Current (A)` - Left motor stator current
- `Shooter/Middle Stator Current (A)` - Middle motor stator current
- `Shooter/Right Stator Current (A)` - Right motor stator current

**Tunable Parameters:**
- `Shooter/Target RPM` - Target RPM for spin-up
- `Shooter/Test Voltage %` - Voltage percentage for testing (default: 0.3)
- `Shooter/Left KP` - Left motor KP gain (default: 0.5)
- `Shooter/KI` - Integral gain (default: 2.0)
- `Shooter/KD` - Derivative gain (default: 0.0)

### Intake

**Read-Only Displays:**
- `Intake/Pivot Angle (degrees)` - Current pivot angle in degrees
- `Intake/Roller RPM` - Roller motor RPM
- `Intake/Pivot Stator Current (A)` - Pivot motor stator current
- `Intake/Roller Stator Current (A)` - Roller motor stator current
- `Intake/Is Homed` - Whether the intake has been homed

**Tunable Parameters:**
- `Intake/Intake Speed %` - Intake roller speed percentage (default: 0.8)
- `Intake/Test Voltage %` - Voltage percentage for testing (default: 0.3)
- `Intake/Pivot KP` - Pivot motor KP gain (default: 300)
- `Intake/Pivot KI` - Pivot motor KI gain (default: 0)
- `Intake/Pivot KD` - Pivot motor KD gain (default: 0)
- `Intake/Homing Voltage %` - Voltage for homing operation (default: 0.1)

### Hood

**Read-Only Displays:**
- `Hood/Current Position` - Current servo position (0.0-1.0)
- `Hood/Target Position` - Target servo position (0.0-1.0)
- `Hood/Within Tolerance` - Whether position is within tolerance

**Tunable Parameters:**
- `Hood/Min Position` - Minimum allowed servo position (default: 0.01)
- `Hood/Max Position` - Maximum allowed servo position (default: 0.77)
- `Hood/Position Tolerance` - Position tolerance band (default: 0.01)
- `Hood/Test Position` - Test position value for manual testing

### Hanger

**Read-Only Displays:**
- `Hanger/Extension (inches)` - Current extension in inches
- `Hanger/Supply Current (A)` - Motor supply current
- `Hanger/Is Homed` - Whether the hanger has been homed

**Tunable Parameters:**
- `Hanger/Test Voltage %` - Voltage percentage for testing (default: 0.3)
- `Hanger/KP` - Motor KP gain (default: 10)
- `Hanger/KI` - Motor KI gain (default: 0)
- `Hanger/KD` - Motor KD gain (default: 0)
- `Hanger/Homing Voltage %` - Voltage for homing (default: -0.05)
- `Hanger/Homing Current Threshold (A)` - Current threshold for homing detection (default: 0.4 A)

### Floor

**Read-Only Displays:**
- `Floor/Current RPM` - Current motor RPM
- `Floor/Stator Current (A)` - Stator current draw
- `Floor/Supply Current (A)` - Supply current draw

**Tunable Parameters:**
- `Floor/Feed Speed %` - Feed speed percentage (default: 0.83)
- `Floor/Test Voltage %` - Voltage percentage for testing (default: 0.3)

## Using Tuning Values in Commands

To use the tunable parameters in your commands, access them through the tuning classes:

### Example: Using Shooter Target RPM

```java
// In a command or subsystem
double targetRPM = SubsystemTuning.ShooterTuning.getTargetRPM();
shooter.setRPM(targetRPM);
```

### Example: Using Intake Homing Voltage

```java
// In a command
double homingVoltage = SubsystemTuning.IntakeTuning.getHomingPercentOutput();
intake.setPivotPercentOutput(homingVoltage);
```

### Example: Using Hanger PID Gains

```java
// When configuring PID
double kP = SubsystemTuning.HangerTuning.getKP();
double kI = SubsystemTuning.HangerTuning.getKI();
double kD = SubsystemTuning.HangerTuning.getKD();
// Apply to motor configuration...
```

## Workflow

1. **Initialize** during robot startup
2. **Observe** real-time values on SmartDashboard during testing
3. **Adjust** tuning parameters using the SmartDashboard interface
4. **Record** the working values back to `Constants.java` once finalized
5. **Verify** smooth operation and mechanical function

## Key Benefits

- **No Recompilation:** Change values on the fly without rebuilding
- **Real-Time Feedback:** See motor RPM, current draw, and positions live
- **Systematic Testing:** All subsystems organized in one place
- **Safe Testing:** 30% voltage default for initial safe testing
- **Current Monitoring:** Track motor loads to detect mechanical issues
- **PID Tuning:** Adjust gains interactively for motion control

## Safety Notes

- Default test voltages are set to 30% for safe low-speed operation
- Always verify mechanical clearances before testing
- Monitor current draw for signs of mechanical binding or issues
- Start with low voltages and gradually increase
- Use deadbands to prevent unintended motion from stick drift

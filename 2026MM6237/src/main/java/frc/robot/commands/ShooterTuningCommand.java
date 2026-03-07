package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;

/**
 * Tuning command for dialing in shooter RPM, hood position at various distances.
 * 
 * HOW TO USE:
 * 1. Hold DPad UP on operator controller (enters tuning mode)
 * 2. Position robot at a known distance from hub
 * 3. Adjust "Tuning/Shooter RPM" in SmartDashboard
 * 4. Adjust "Tuning/Hood Position" in SmartDashboard (0.01 - 0.77)
 * 5. Pull RIGHT TRIGGER on operator controller to fire test shot
 *    (feeds while trigger is held AND shooter is at speed)
 * 6. Observe result and adjust
 * 7. Record working values
 */
public class ShooterTuningCommand extends Command {
    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final Floor floor;
    private final LimelightSubsystem6237 limelight;
    private final CommandXboxController operatorController;
    
    // Tuning values - adjusted via SmartDashboard
    private double targetRPM = 3000;
    private double targetHoodPosition = 0.5;
    
    // State
    private boolean wasAtSpeed = false;
    private boolean isFiring = false;
    
    public ShooterTuningCommand(Shooter shooter, Hood hood, Feeder feeder, Floor floor,
                                 LimelightSubsystem6237 limelight, CommandXboxController operatorController) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.floor = floor;
        this.limelight = limelight;
        this.operatorController = operatorController;
        
        // Require ALL shooting subsystems so nothing else can interrupt us
        addRequirements(shooter, hood, feeder, floor);
        
        // Only set defaults if keys don't exist yet - never overwrite user's values
        if (!SmartDashboard.containsKey("Tuning/Shooter RPM")) {
            SmartDashboard.putNumber("Tuning/Shooter RPM", targetRPM);
        }
        if (!SmartDashboard.containsKey("Tuning/Hood Position")) {
            SmartDashboard.putNumber("Tuning/Hood Position", targetHoodPosition);
        }
    }
    
    @Override
    public void initialize() {
        // Read current dashboard values so we never overwrite what the user set
        targetRPM = SmartDashboard.getNumber("Tuning/Shooter RPM", targetRPM);
        targetHoodPosition = SmartDashboard.getNumber("Tuning/Hood Position", targetHoodPosition);
        wasAtSpeed = false;
        isFiring = false;
        SmartDashboard.putString("Tuning/Status", "TUNING ACTIVE - Adjust values, RT to fire");
        SmartDashboard.putBoolean("Tuning/At Speed", false);
        SmartDashboard.putBoolean("Tuning/Firing", false);
    }
    
    @Override
    public void execute() {
        // Read tuning values from SmartDashboard
        targetRPM = SmartDashboard.getNumber("Tuning/Shooter RPM", targetRPM);
        targetHoodPosition = SmartDashboard.getNumber("Tuning/Hood Position", targetHoodPosition);
        
        // Apply shooter RPM
        shooter.setRPM(targetRPM);
        
        // Apply hood position
        hood.setPosition(targetHoodPosition);
        
        // Get current distance from Limelight
        double distance = limelight.getLastHubDistance();
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        
        // Check if shooter is at speed
        boolean atSpeed = shooter.isVelocityWithinTolerance();
        
        // Check if operator is pulling right trigger to fire
        boolean triggerHeld = operatorController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold;
        
        // Fire: Run feeder and floor when trigger held AND shooter at speed
        if (triggerHeld && atSpeed) {
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
            isFiring = true;
        } else {
            feeder.setPercentOutput(0);
            floor.set(Floor.Speed.STOP);
            isFiring = false;
        }
        
        // Update status displays
        SmartDashboard.putBoolean("Tuning/At Speed", atSpeed);
        SmartDashboard.putBoolean("Tuning/Firing", isFiring);
        SmartDashboard.putNumber("Tuning/Current Distance (m)", distance);
        SmartDashboard.putBoolean("Tuning/Hub Visible", hubVisible);
        
        // Display actual motor RPMs for comparison
        SmartDashboard.putNumber("Tuning/Actual Left RPM", shooter.getLeftMotorRPM());
        SmartDashboard.putNumber("Tuning/Actual Middle RPM", shooter.getMiddleMotorRPM());
        SmartDashboard.putNumber("Tuning/Actual Right RPM", shooter.getRightMotorRPM());
        
        // Display hood position
        SmartDashboard.putNumber("Tuning/Target Hood Position", targetHoodPosition);
        SmartDashboard.putNumber("Tuning/Actual Hood Position", hood.getCurrentPosition());
        
        // Status message
        if (isFiring) {
            SmartDashboard.putString("Tuning/Status", ">>> FIRING <<< RPM=" + (int)targetRPM + " Hood=" + String.format("%.2f", targetHoodPosition));
        } else if (triggerHeld && !atSpeed) {
            SmartDashboard.putString("Tuning/Status", "TRIGGER HELD - Waiting for speed...");
        } else if (!hubVisible) {
            SmartDashboard.putString("Tuning/Status", "Tuning | No hub visible | RT to fire");
        } else if (!atSpeed) {
            SmartDashboard.putString("Tuning/Status", "SPINNING UP @ " + String.format("%.1f", distance) + "m | RT to fire");
        } else {
            SmartDashboard.putString("Tuning/Status", "READY @ " + String.format("%.1f", distance) + "m | RT to fire");
        }
        
        // Log when we reach speed
        if (atSpeed && !wasAtSpeed) {
            SmartDashboard.putString("Tuning/Last Event", "Reached target RPM");
        }
        wasAtSpeed = atSpeed;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.setPercentOutput(0);
        floor.set(Floor.Speed.STOP);
        SmartDashboard.putString("Tuning/Status", "TUNING STOPPED");
        SmartDashboard.putBoolean("Tuning/At Speed", false);
        SmartDashboard.putBoolean("Tuning/Firing", false);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until DPad UP is released
    }
}

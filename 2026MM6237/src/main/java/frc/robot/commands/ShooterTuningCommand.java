package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;

/**
 * Tuning command for dialing in shooter RPM, hood position, and feeder speed at various distances.
 * 
 * HOW TO USE:
 * 1. Enable this command (bind to a button)
 * 2. Position robot at a known distance from hub (use Limelight/Hub Distance (m) readout)
 * 3. Adjust "Tuning/Shooter RPM" in SmartDashboard
 * 4. Adjust "Tuning/Hood Position" in SmartDashboard (0.0 - 1.0)
 * 5. Press operator trigger to fire test shot
 * 6. Observe result and adjust
 * 7. When shot is good, record values in the table below
 * 
 * SUGGESTED TEST DISTANCES (meters):
 * - 1.5m (close)
 * - 2.5m (medium-close)  
 * - 3.5m (medium)
 * - 4.5m (medium-far)
 * - 5.5m (far)
 * 
 * Record your findings here:
 * Distance | RPM  | Hood | Notes
 * ---------|------|------|------
 * 1.5m     |      |      |
 * 2.5m     |      |      |
 * 3.5m     |      |      |
 * 4.5m     |      |      |
 * 5.5m     |      |      |
 */
public class ShooterTuningCommand extends Command {
    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final LimelightSubsystem6237 limelight;
    
    // Tuning values - adjusted via SmartDashboard
    private double targetRPM = 3000;
    private double targetHoodPosition = 0.5;
    private double feederSpeed = 0.8;
    
    // State
    private boolean wasAtSpeed = false;
    
    public ShooterTuningCommand(Shooter shooter, Hood hood, Feeder feeder, LimelightSubsystem6237 limelight) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.limelight = limelight;
        
        addRequirements(shooter, hood);
        // Note: Don't require feeder - we'll control it separately for test shots
        
        // Initialize SmartDashboard values
        SmartDashboard.putNumber("Tuning/Shooter RPM", targetRPM);
        SmartDashboard.putNumber("Tuning/Hood Position", targetHoodPosition);
        SmartDashboard.putNumber("Tuning/Feeder Speed", feederSpeed);
    }
    
    @Override
    public void initialize() {
        wasAtSpeed = false;
        SmartDashboard.putString("Tuning/Status", "TUNING ACTIVE - Adjust values");
        SmartDashboard.putBoolean("Tuning/At Speed", false);
    }
    
    @Override
    public void execute() {
        // Read tuning values from SmartDashboard
        targetRPM = SmartDashboard.getNumber("Tuning/Shooter RPM", targetRPM);
        targetHoodPosition = SmartDashboard.getNumber("Tuning/Hood Position", targetHoodPosition);
        feederSpeed = SmartDashboard.getNumber("Tuning/Feeder Speed", feederSpeed);
        
        // Apply shooter RPM
        shooter.setRPM(targetRPM);
        
        // Apply hood position
        hood.setPosition(targetHoodPosition);
        
        // Get current distance from Limelight
        double distance = limelight.getLastHubDistance();
        boolean hubVisible = limelight.isHubCurrentlyVisible();
        
        // Check if shooter is at speed
        boolean atSpeed = shooter.isVelocityWithinTolerance();
        
        // Update status displays
        SmartDashboard.putBoolean("Tuning/At Speed", atSpeed);
        SmartDashboard.putNumber("Tuning/Current Distance (m)", distance);
        SmartDashboard.putBoolean("Tuning/Hub Visible", hubVisible);
        
        // Display actual motor RPMs for comparison
        SmartDashboard.putNumber("Tuning/Actual Left RPM", shooter.getLeftMotorRPM());
        SmartDashboard.putNumber("Tuning/Actual Middle RPM", shooter.getMiddleMotorRPM());
        SmartDashboard.putNumber("Tuning/Actual Right RPM", shooter.getRightMotorRPM());
        
        // Display hood position
        SmartDashboard.putNumber("Tuning/Actual Hood Position", hood.getCurrentPosition());
        
        // Status message
        if (!hubVisible) {
            SmartDashboard.putString("Tuning/Status", "NO HUB VISIBLE - Aim at target");
        } else if (!atSpeed) {
            SmartDashboard.putString("Tuning/Status", "SPINNING UP... " + String.format("%.0f", distance) + "m");
        } else {
            SmartDashboard.putString("Tuning/Status", "READY TO FIRE @ " + String.format("%.1f", distance) + "m");
        }
        
        // Log when we reach speed (for timing feedback)
        if (atSpeed && !wasAtSpeed) {
            SmartDashboard.putString("Tuning/Last Event", "Reached target RPM");
        }
        wasAtSpeed = atSpeed;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putString("Tuning/Status", "TUNING STOPPED");
        SmartDashboard.putBoolean("Tuning/At Speed", false);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}

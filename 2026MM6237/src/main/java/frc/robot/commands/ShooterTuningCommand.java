package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
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
 * Uses a dedicated "Tune" NetworkTables table (NOT SmartDashboard) to avoid
 * any conflicts with SubsystemTuning or other code writing to SmartDashboard.
 * 
 * In Shuffleboard or OutlineViewer, look for the "Tune" table.
 * In SmartDashboard, the entries appear as "Tune/RPM" and "Tune/Hood".
 * 
 * HOW TO USE:
 * 1. Hold DPad UP on operator controller → enters tuning mode, shooter spins up
 * 2. In dashboard, change "Tune/RPM" and "Tune/Hood" to desired values
 *    (These are the ONLY two values you edit - everything else is display-only)
 * 3. Pull RIGHT TRIGGER on operator controller to fire test shot
 *    (feeds while trigger is held AND shooter is at speed)
 * 4. Observe result, adjust values, repeat
 * 5. Record working values for each distance
 * 
 * STARTING VALUES (estimates based on existing interpolation data):
 * Distance | RPM  | Hood | Notes
 * ---------|------|------|------
 * 1.5m     | 3100 | 0.25 | Close range, low hood
 * 2.5m     | 3500 | 0.40 | Medium-close
 * 3.5m     | 3800 | 0.50 | Medium
 * 4.5m     | 4100 | 0.58 | Medium-far
 */
public class ShooterTuningCommand extends Command {
    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final Floor floor;
    private final LimelightSubsystem6237 limelight;
    private final CommandXboxController operatorController;
    
    // Dedicated NT table - completely separate from SmartDashboard
    private static final NetworkTable tuneTable = NetworkTableInstance.getDefault().getTable("Tune");
    
    // USER-EDITABLE inputs: we only SUBSCRIBE (read), never write to these after init
    private final DoubleSubscriber rpmSub;
    private final DoubleSubscriber hoodSub;
    
    // DISPLAY-ONLY outputs: we only PUBLISH (write), user should not edit these
    private final DoublePublisher distancePub;
    private final DoublePublisher actualLeftRpmPub;
    private final DoublePublisher actualMidRpmPub;
    private final DoublePublisher actualRightRpmPub;
    private final DoublePublisher actualHoodPub;
    private final DoublePublisher commandedRpmPub;
    private final DoublePublisher commandedHoodPub;
    private final StringPublisher statusPub;
    private final BooleanPublisher atSpeedPub;
    private final BooleanPublisher firingPub;
    private final BooleanPublisher hubVisiblePub;
    
    // Only publish initial defaults once per robot boot
    private static boolean defaultsPublished = false;
    
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
        
        // Require ALL shooting subsystems so PrepareStaticShotCommand cannot interrupt us
        addRequirements(shooter, hood, feeder, floor);
        
        // Subscribe to user-editable inputs
        rpmSub = tuneTable.getDoubleTopic("RPM").subscribe(3100);
        hoodSub = tuneTable.getDoubleTopic("Hood").subscribe(0.25);
        
        // Create publishers for display-only status
        distancePub = tuneTable.getDoubleTopic("Distance (m)").publish();
        actualLeftRpmPub = tuneTable.getDoubleTopic("Actual Left RPM").publish();
        actualMidRpmPub = tuneTable.getDoubleTopic("Actual Mid RPM").publish();
        actualRightRpmPub = tuneTable.getDoubleTopic("Actual Right RPM").publish();
        actualHoodPub = tuneTable.getDoubleTopic("Actual Hood Pos").publish();
        commandedRpmPub = tuneTable.getDoubleTopic("Commanded RPM").publish();
        commandedHoodPub = tuneTable.getDoubleTopic("Commanded Hood").publish();
        statusPub = tuneTable.getStringTopic("Status").publish();
        atSpeedPub = tuneTable.getBooleanTopic("At Speed").publish();
        firingPub = tuneTable.getBooleanTopic("Firing").publish();
        hubVisiblePub = tuneTable.getBooleanTopic("Hub Visible").publish();
        
        // Publish starting defaults ONCE per robot boot so the keys appear in the dashboard
        // After this, we never write to RPM or Hood again - user's edits will stick
        if (!defaultsPublished) {
            DoublePublisher rpmInit = tuneTable.getDoubleTopic("RPM").publish();
            DoublePublisher hoodInit = tuneTable.getDoubleTopic("Hood").publish();
            rpmInit.set(3100);
            hoodInit.set(0.25);
            // Don't close these publishers - let the values persist
            defaultsPublished = true;
        }
    }
    
    @Override
    public void initialize() {
        wasAtSpeed = false;
        isFiring = false;
        statusPub.set("TUNING ACTIVE - Edit Tune/RPM and Tune/Hood, pull RT to fire");
        atSpeedPub.set(false);
        firingPub.set(false);
    }
    
    @Override
    public void execute() {
        // Read user's tuning values - subscriptions ONLY READ, never write back
        double targetRPM = rpmSub.get(3100);
        double targetHoodPosition = hoodSub.get(0.25);
        
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
        
        // Publish display-only status (completely separate keys from RPM and Hood)
        commandedRpmPub.set(targetRPM);
        commandedHoodPub.set(targetHoodPosition);
        atSpeedPub.set(atSpeed);
        firingPub.set(isFiring);
        distancePub.set(distance);
        hubVisiblePub.set(hubVisible);
        actualLeftRpmPub.set(shooter.getLeftMotorRPM());
        actualMidRpmPub.set(shooter.getMiddleMotorRPM());
        actualRightRpmPub.set(shooter.getRightMotorRPM());
        actualHoodPub.set(hood.getCurrentPosition());
        
        // Status message
        if (isFiring) {
            statusPub.set(">>> FIRING <<< RPM=" + (int)targetRPM + " Hood=" + String.format("%.2f", targetHoodPosition));
        } else if (triggerHeld && !atSpeed) {
            statusPub.set("TRIGGER HELD - Waiting for speed...");
        } else if (!atSpeed) {
            statusPub.set("SPINNING UP | RPM=" + (int)targetRPM + " Hood=" + String.format("%.2f", targetHoodPosition) + " | RT to fire");
        } else {
            statusPub.set("READY @ " + String.format("%.1f", distance) + "m | RT to fire");
        }
        
        // Log when we reach speed
        if (atSpeed && !wasAtSpeed) {
            tuneTable.getStringTopic("Last Event").publish().set("Reached target RPM");
        }
        wasAtSpeed = atSpeed;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.setPercentOutput(0);
        floor.set(Floor.Speed.STOP);
        statusPub.set("TUNING STOPPED");
        atSpeedPub.set(false);
        firingPub.set(false);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until DPad UP is released
    }
}

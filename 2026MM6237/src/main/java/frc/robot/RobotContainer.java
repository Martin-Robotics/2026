// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.generated.TunerConstants;
// import frc.robot.util.SubsystemTuning; // Commented out after testing phase
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.HubMonitor;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.auto.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();

    private final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    // private final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController operator =new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
      private final SendableChooser<Command> autoChooser;

    private final LimelightSubsystem6237 limelight = new LimelightSubsystem6237(drivetrain);
    private final LEDSubsystem m_leds = new LEDSubsystem();
    private final HubMonitor m_hubMonitor = new HubMonitor();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Register PathPlanner NamedCommands for autonomous routines
      registerNamedCommands();

      intake.homingCommand().schedule();
      intake.setDefaultCommand(
        Commands.run(() -> intake.set(Intake.Position.STOWED), intake)
            .onlyIf(() -> intake.isHomed())
            .withName("HoldStowed"));
      
      // Wire limelight into LED subsystem for target-lock indication
      m_leds.setLimelightSubsystem(limelight);

      // Configure the trigger bindings
      configureBindings();
      autoChooser = AutoBuilder.buildAutoChooser("DefaultAuto");
      SmartDashboard.putData("Auto Mode", autoChooser);
      
      // Initialize dashboard values so they appear on startup
      initializeDashboard();
      
      // Enable target tuning dashboard for field calibration
      // Comment out or call limelight.removeTargetTuningDashboard() for competition
      limelight.initializeTargetTuningDashboard();
    }
    
    /**
     * Initialize the minimal set of SmartDashboard values so they're visible on startup.
     */
    private void initializeDashboard() {
      SmartDashboard.putNumber("Hub Distance (m)", 0.0);
      SmartDashboard.putBoolean("Hub Locked", false);
      SmartDashboard.putNumber("Hub Tag ID", 0);
      SmartDashboard.putBoolean("AimAtHub Mode", false);
      SmartDashboard.putString("Hub/Active Hub", "Both");
      SmartDashboard.putBoolean("Hub/Our Hub Active", true);
      SmartDashboard.putString("Hub/Status", "");
      SmartDashboard.putNumber("Hub/Countdown (s)", 0.0);
      SmartDashboard.putNumber("Match Time (s)", 0.0);
      SmartDashboard.putNumber("Auto/Speed %", Constants.CommandSwerveDrivetrain.kAutoSpeedScaleFactor);
    }

    /**
     * Register all autonomous commands with PathPlanner's NamedCommands system.
     * These commands can be used in PathPlanner auto files via their registered names.
     */
    private void registerNamedCommands() {
      // Intake commands
      NamedCommands.registerCommand("PrepareForIntake", new PrepareForIntake(intake));
      NamedCommands.registerCommand("RunIntake", new RunIntake(intake));
      NamedCommands.registerCommand("StopIntake", new StopIntake(intake));
      NamedCommands.registerCommand("ExtendIntakeArm", new ExtendIntakeArm(intake));
      NamedCommands.registerCommand("RetractIntakeArm", new RetractIntakeArm(intake));
      NamedCommands.registerCommand("StartIntakeRollers", new StartIntakeRollers(intake));
      NamedCommands.registerCommand("StopIntakeRollers", new StopIntakeRollers(intake));
      
      // Shooter commands - use Autonomous versions with timeout/duration
      // PrepareToFire: aims at hub using Limelight TX with PD control, auto-ends when aimed or timeout
      NamedCommands.registerCommand("PrepareToFire", new PrepareToFireAutonomous(limelight, drivetrain, hood));
      // Fire: spins up shooter/hood from interpolation table, feeds when at speed, auto-ends after duration
      NamedCommands.registerCommand("Fire", new FireAutonomous(feeder, shooter, hood, floor, limelight));
      
      // Climb commands
      NamedCommands.registerCommand("PrepareToClimb", new PrepareToClimb(hanger));
      NamedCommands.registerCommand("PrepareToClimbLeft", new PrepareToClimbLeft(hanger));
      NamedCommands.registerCommand("PrepareToClimbRight", new PrepareToClimbRight(hanger));
      NamedCommands.registerCommand("Climb", new Climb(hanger));
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      // DriverMapping6237MR.mapXboxController(driver, drivetrain, NetworkTableInstance.getDefault().getTable("limelight"));
      DriverController.mapXboxController(driver, drivetrain, null, shooter, limelight, hood, m_leds, m_hubMonitor);
      OperatorController.mapXboxController(operator, feeder, shooter, intake, hood, hanger, floor, limelight, drivetrain, driver);
      // OperatorController.mapXboxControllerTestInputs(operator, feeder, shooter, intake, hood, hanger, floor);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
      // return new Command(){};
    }

    public void getSimPeriodic(Field2d field) {
      field.setRobotPose(drivetrain.getState().Pose);
    }

    // ======================== SUBSYSTEM GETTERS ========================

    public Feeder getFeeder() {
      return feeder;
    }

    public Shooter getShooter() {
      return shooter;
    }

    public Intake getIntake() {
      return intake;
    }

    public Hood getHood() {
      return hood;
    }

    public Hanger getHanger() {
      return hanger;
    }

    public Floor getFloor() {
      return floor;
    }

    public LEDSubsystem getLEDs() {
      return m_leds;
    }
}

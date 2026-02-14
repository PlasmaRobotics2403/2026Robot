package frc.robot;

import static frc.robot.Constants.Cameras.camera0Name;
import static frc.robot.Constants.Cameras.camera1Name;
import static frc.robot.Constants.Cameras.robotToCamera0;
import static frc.robot.Constants.Cameras.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SimCameras;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.AutopilotCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RunIndexterDutyCycle;
import frc.robot.commands.TestTurretFollowCommand;
import frc.robot.commands.TestTurretToPosCommand;
import frc.robot.commands.TurretFlipCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TestMotorSubsystem;
import frc.robot.subsystems.TestMotorSubsystemTwo;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.flywheel_example.Flywheel;
import frc.robot.subsystems.flywheel_example.FlywheelIO;
import frc.robot.subsystems.flywheel_example.FlywheelIOSim;
import frc.robot.subsystems.imu.ImuIO;
import frc.robot.subsystems.imu.ImuIOPigeon2;
import frc.robot.subsystems.imu.ImuIOSim;
import frc.robot.subsystems.shooter.SimTurretSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.CameraSweepEvaluator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GetJoystickValue;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIPowerMonitor;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  /** Define the Driver and, optionally, the Operator/Co-Driver Controllers */
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  final CommandXboxController driverController = new CommandXboxController(0); // Main Driver

  final CommandXboxController operatorController = new CommandXboxController(1); // Second Operator
  final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  // These two are needed for the Sweep evaluator for camera FOV simulation
  final CommandJoystick joystick3 = new CommandJoystick(3); //  Joystick for CamersSweepEvaluator
  private final CameraSweepEvaluator sweep;

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controls
  private final Drive m_drivebase;

  private final ImuIO m_imu;
  private final Flywheel m_flywheel;
  // Test mechanism subsystems
  private final TestTurretSubsystem m_testTurret = new TestTurretSubsystem();

  // "Competition-style" turret logic sandbox. This is separated from TestTurretSubsystem so the
  // test turret remains available as a fallback.
  private final TurretSubsystem m_turretSmSandbox =
      new SimTurretSubsystem(Units.degreesToRadians(-180.0), Units.degreesToRadians(180.0));

  // Simple test motor subsystem
  private final TestMotorSubsystem m_testMotor = new TestMotorSubsystem(26);
  private final TestMotorSubsystemTwo m_testMotorTwo = new TestMotorSubsystemTwo(24);

  private final IndexerSubsystem m_indexer = new IndexerSubsystem();

  // Intake subsystem
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // ... Add additional subsystems here (e.g., elevator, arm, etc.)

  // These are "Virtual Subsystems" that report information but have no motors
  @SuppressWarnings("unused")
  private final Accelerometer m_accel;

  @SuppressWarnings("unused")
  private final RBSIPowerMonitor m_power;

  @SuppressWarnings("unused")
  private final Vision m_vision;

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);

  // EXAMPLE TUNABLE FLYWHEEL SPEED INPUT FROM DASHBOARD
  @SuppressWarnings("unused")
  private final LoggedTunableNumber flywheelSpeedInput =
      new LoggedTunableNumber("Flywheel Speed", 1500.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {

    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // YAGSL drivebase, get config from deploy directory

        // Get the IMU instance
        switch (SwerveConstants.kImuType) {
          case "pigeon2":
            m_imu = new ImuIOPigeon2();
            break;
          default:
            throw new RuntimeException("Invalid IMU type");
        }

        m_drivebase = new Drive(m_imu);
        m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_vision =
            switch (Constants.getVisionType()) {
              case PHOTON ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOPhotonVision(camera0Name, robotToCamera0),
                      new VisionIOPhotonVision(camera1Name, robotToCamera1));
              case NONE ->
                  new Vision(
                      m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
              default -> null;
            };
        m_accel = new Accelerometer(m_imu);
        sweep = null;
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_imu = new ImuIOSim();
        m_drivebase = new Drive(m_imu);
        m_flywheel = new Flywheel(new FlywheelIOSim() {});
        m_vision =
            new Vision(
                m_drivebase::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drivebase::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drivebase::getPose));
        m_accel = new Accelerometer(m_imu);

        // CameraSweepEvaluator Construct
        // 1) Create the vision simulation world
        VisionSystemSim visionSim = new VisionSystemSim("CameraSweepWorld");
        // 2) Add AprilTags (field layout)
        visionSim.addAprilTags(FieldConstants.aprilTagLayout);
        // 3) Create two simulated cameras
        // Create PhotonCamera objects (names must match VisionIOPhotonVisionSim)
        PhotonCamera camera1 = new PhotonCamera(camera0Name);
        PhotonCamera camera2 = new PhotonCamera(camera1Name);

        // Wrap them with simulation + properties (2026 API)
        PhotonCameraSim cam1 = new PhotonCameraSim(camera1, SimCameras.kSimCamera1Props);
        PhotonCameraSim cam2 = new PhotonCameraSim(camera2, SimCameras.kSimCamera2Props);

        // 4) Register cameras with the sim
        visionSim.addCamera(cam1, Transform3d.kZero);
        visionSim.addCamera(cam2, Transform3d.kZero);
        // 5) Create the sweep evaluator
        sweep = new CameraSweepEvaluator(visionSim, cam1, cam2);

        break;

      default:
        // Replayed robot, disable IO implementations
        m_imu = new ImuIOSim();
        m_drivebase = new Drive(m_imu);
        m_flywheel = new Flywheel(new FlywheelIO() {});
        m_vision =
            new Vision(m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_accel = new Accelerometer(m_imu);
        sweep = null;
        break;
    }

    // In addition to the initial battery capacity from the Dashbaord, ``RBSIPowerMonitor`` takes
    // all the non-drivebase subsystems for which you wish to have power monitoring; DO NOT
    // include ``m_drivebase``, as that is automatically monitored.
    m_power = new RBSIPowerMonitor(batteryCapacity, m_flywheel);

    // Ensure Vision.periodic() runs every loop (even if no command requires Vision yet)
    if (m_vision != null) {
      CommandScheduler.getInstance().registerSubsystem(m_vision);
    }

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case MANUAL:
        // This is where the "Leave Auto" will go
        // ...
        // Set the others to null
        autoChooserPathPlanner = null;
        break;

      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Define Auto commands
    defineAutoCommands();
    // Define SysIs Routines
    definesysIdRoutines();
    // Configure the button and trigger bindings
    configureBindings();
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner */
  private void defineAutoCommands() {

    // NamedCommands.registerCommand("Zero", Commands.runOnce(() -> m_drivebase.zero()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // Send the proper joystick input based on driver preference -- Set this in `Constants.java`
    GetJoystickValue driveStickY;
    GetJoystickValue driveStickX;
    GetJoystickValue turnStickX;
    switch (OperatorConstants.kDriveStyle) {
      case GAMER:
        driveStickY = driverController::getRightY;
        driveStickX = driverController::getRightX;
        turnStickX = driverController::getLeftX;
        break;
      default: // Includes case TANK
        driveStickY = driverController::getLeftY;
        driveStickX = driverController::getLeftX;
        turnStickX = driverController::getRightX;
    }

    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driveStickY.value(),
            () -> -driveStickX.value(),
            () -> -turnStickX.value()));

    // ** Example Commands -- Remap, remove, or change as desired **
    // Hold B button to run the test motor (duty cycle)
    driverController.b().whileTrue(new RunIndexterDutyCycle(m_indexer, 0.5, 0.5));

    // Press X button --> Stop with wheels in X-Lock position
    driverController.x().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));

    // Press Y button --> Manually Re-Zero the Gyro
    driverController.y().onTrue(new TestTurretToPosCommand(m_testTurret, 170));
    driverController.x().onTrue(new TestTurretToPosCommand(m_testTurret, -170));

    // Press RIGHT BUMPER --> Deploy intake + run rollers; release -> stow + stop
    driverController.rightBumper().whileTrue(new IntakeCommand(m_intake));

    // Press LEFT BUMPER --> Drive to a pose 10 feet closer to the BLUE ALLIANCE wall
    driverController
        .leftTrigger()
        .whileTrue(
            Commands.defer(
                () -> {
                  // New pose 2 feet closer to BLUE ALLIANCE wall
                  Pose2d pose =
                      m_drivebase
                          .getPose()
                          .transformBy(
                              new Transform2d(Units.feetToMeters(-10.0), 0.0, Rotation2d.kZero));

                  // Alternatively, you could define a pose in a separate module and call it here.
                  //
                  // Example from 2025 Reefscape:
                  // --------
                  // pose = ReefPoses.kBluePoleE;

                  return AutopilotCommands.runAutopilot(m_drivebase, pose);
                },
                Set.of(m_drivebase)));

    // Press POV LEFT to nudge the robot left
    driverController
        .povLeft()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(0.), Units.inchesToMeters(11.0), 0.));
                },
                // Stop when command ended
                m_drivebase::stop,
                m_drivebase));

    // Hold A to aim turret at the AprilTag(tx -> 0).
    // If no target is visible, the turret holds its current position.
    Command follow = new TestTurretFollowCommand(m_testTurret, m_vision);
    Command flip = new TurretFlipCommand(m_testTurret);

    Command followFlipLoop =
        Commands.repeatingSequence(
            follow.until(m_testTurret::isAtLimit), flip, Commands.waitSeconds(1));
    driverController.a().whileTrue(followFlipLoop);

    // State-machine turret follow (separate binding) - only enable during tuning so it can't
    // surprise you mid-match.
    if (m_vision != null) {
      if (Constants.tuningMode) {
        driverController.b();
      }
    }

    if (Constants.getMode() == Mode.SIM) {
      // IN SIMULATION ONLY:
      // Double-press the A button on Joystick3 to run the CameraSweepEvaluator
      // Use WPILib's built-in double-press binding
      joystick3
          .button(1)
          .multiPress(2, 0.2)
          .onTrue(
              Commands.runOnce(
                  () -> {
                    try {
                      sweep.runFullSweep(
                          Filesystem.getOperatingDirectory()
                              .toPath()
                              .resolve("camera_sweep.csv")
                              .toString());
                    } catch (Exception e) {
                      e.printStackTrace();
                    }
                  }));
    }
  }

  public Command getManualAuto() {
    // NOTE:
    //
    // For teams not using PathPlanner, this auto may be used to simply shoot the pre-loaded fuel
    // into the HUB during AUTO.  Since shooters are beyond the scope of Az-RBSI, you will have to
    // write your own command and call it here.

    // Replace Commands.none() with your command that shoots fuel into the HUB.
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
  }

  /** Set the motor neutral mode to BRAKE / COAST for T/F */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = Constants.getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /** Drivetrain getter method */
  public Drive getDrivebase() {
    return m_drivebase;
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == AutoType.PATHPLANNER) {
      // Simple test auto: follow a single PathPlanner path named "Example Path"
      autoChooserPathPlanner.addOption(
          "Test: Example Path",
          Commands.defer(
              () -> {
                try {
                  return AutoBuilder.followPath(PathPlannerPath.fromPathFile("New Path"));
                } catch (Exception e) {
                  // If the path isn't present yet, keep the robot safe and do nothing.
                  // (Also avoids RobotContainer failing to construct.)
                  return Commands.none();
                }
              },
              Set.of(m_drivebase)));

      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Example Flywheel SysId Characterization
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Quasistatic Forward)",
          m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Quasistatic Reverse)",
          m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Dynamic Forward)",
          m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Dynamic Reverse)",
          m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }
}

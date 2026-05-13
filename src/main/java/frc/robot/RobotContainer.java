package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeBallsCommand;
import frc.robot.commands.IntakePulseCommand;
import frc.robot.commands.IntakeStowCommand;
import frc.robot.commands.OutakeBallsCommand;
import frc.robot.commands.RunIndexterDutyCycle;
import frc.robot.commands.ShootAutoCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootFromDistanceToHubCommand;
import frc.robot.commands.ShootFromDistanceToHubCommandAuto;
import frc.robot.commands.ShuttleShot;
import frc.robot.commands.TestTurretToPosCommand;
import frc.robot.commands.TurretFollowCommand;
import frc.robot.commands.ZeroTurretCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.TurretGridSelector;
import frc.robot.util.TurretGridSelector.GridTarget;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private static final String FAR_MID_SCORE_AUTO_FILE = "Far Mid Score Auto";
    private static final String NEAR_MID_SCORE_AUTO_FILE = "Near Mid Score Auto";
    private static final String DRIVE_FORWARD_AUTO_FILE = "Drive Forward Auto";
    private static final String SHOOT_ONLY_AUTO_NAME = "Shoot Only";

    public final LEDs leds = new LEDs();
    private final Vision vision;
    private final Drive drive;
    private final Shooter shooter;
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private SwerveDriveSimulation driveSimulation = null;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController navigator = new CommandXboxController(1);

    private final TestTurretSubsystem testTurret = new TestTurretSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final Field2d field = new Field2d();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    private final LoggedDashboardChooser<Command> autoChooser;

    private final VisionIOPhotonVision cam0VisionIO;

    private final SendableChooser<Command> chooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (robotPose) -> {});
                cam0VisionIO = new VisionIOPhotonVision(camera0Name, robotToCamera0, drive);
                vision = new Vision(
                        drive,
                        cam0VisionIO,
                        new VisionIOPhotonVision(camera1Name, robotToCamera1, drive),
                        new VisionIOPhotonVision(camera2Name, robotToCamera2, drive));
                shooter = new Shooter(new ShooterIOTalonFX(), this::calculateDistanceToTargetMeters, drive);
                break;

            case SIM:
                SimulatedArena.overrideInstance(new Arena2026Rebuilt(true));
                driveSimulation =
                        new SwerveDriveSimulation(Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose, drive),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose, drive));
                shooter = new Shooter(new ShooterIOSim(), this::calculateDistanceToTargetMeters, drive);
                cam0VisionIO = new VisionIOPhotonVision(camera0Name, robotToCamera0, drive);
                break;

            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                shooter = new Shooter(new ShooterIO() {}, this::calculateDistanceToTargetMeters, drive);
                cam0VisionIO = new VisionIOPhotonVision(camera0Name, robotToCamera0, drive);
                break;
        }

        registerNamedCommands();
        SmartDashboard.putData("Field", field);

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // autoChooser.addOption("Far Mid Semantic Auto", buildAllianceCorrectFarMidAuto());
        // autoChooser.addOption("Near Mid Semantic Auto", buildAllianceCorrectNearMidAuto());
        autoChooser.addOption("2BlueFar Mid Semantic Auto", buildAllianceCorrect2BlueFarMidAuto());
        autoChooser.addOption("2BlueNear Mid Semantic Auto", buildAllianceCorrect2BlueNearMidAuto());
        autoChooser.addOption("2RedFar Mid Semantic Auto", buildAllianceCorrect2RedFarMidAuto());
        autoChooser.addOption("2RedNear Mid Semantic Auto", buildAllianceCorrect2RedNearMidAuto());

        autoChooser.addOption("Depo Auto", buildDepoAuto());

        autoChooser.addOption("RedNearSweep Score Auto", buildRedNearSweepAuto());
        autoChooser.addOption("RedFarSweep Score Auto", buildRedFarSweepAuto());
        autoChooser.addOption("BlueNearSweep Score Auto", buildBlueNearSweepAuto());
        autoChooser.addOption("BlueFarSweep Score Auto", buildBlueFarSweepAuto());

        autoChooser.addOption("FastRedNearSweep Score Auto", buildFastRedNearSweepAuto());
        autoChooser.addOption("FastRedFarSweep Score Auto", buildFastRedFarSweepAuto());
        autoChooser.addOption("FastBlueNearSweep Score Auto", buildFastBlueNearSweepAuto());
        autoChooser.addOption("FastBlueFarSweep Score Auto", buildFastBlueFarSweepAuto());

        // autoChooser.addOption("Shoot Only Near Blue", buildShootOnlyAutoNearBlue());
        // autoChooser.addOption("Shoot Only Far Blue", buildShootOnlyAutoFarBlue());
        // autoChooser.addOption("Shoot Only Near Red", buildShootOnlyAutoNearRed());
        // autoChooser.addOption("Shoot Only Far Red", buildShootOnlyAutoFarRed());
        autoChooser.addOption("Drive", buildDriveForwardAuto());
        // autoChooser.addOption("Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(chooser);
        configureButtonBindings();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Deploy Intake", new IntakeBallsCommand(intake).withName("Deploy Intake"));

        NamedCommands.registerCommand("Stow Intake", new IntakeStowCommand(intake).withName("Stow Intake"));
        NamedCommands.registerCommand("Pulse Intake", new IntakePulseCommand(intake).withName("Pulse Intake"));

        NamedCommands.registerCommand(
                "Static Shot",
                new ShootAutoCommand(shooter, indexer, drive, testTurret, intake).withName("Static Shot"));

        NamedCommands.registerCommand(
                "Test Static Shot",
                Commands.runOnce(() -> shooter.runShot(0.5, 55), shooter)
                        .andThen(Commands.runOnce(() -> testTurret.setTargetAngleRadians(Math.toRadians(-24))))
                        .andThen(Commands.runOnce(() -> indexer.index(0.5), indexer)));

        NamedCommands.registerCommand(
                "Stop Rollers",
                Commands.runOnce(
                                () -> {
                                    intake.setPivotTargetDegrees(Constants.IntakeConstants.STOW_DEG);
                                    intake.stopRoller();
                                },
                                intake)
                        .withName("Stop Rollers"));

        NamedCommands.registerCommand(
                "Shoot",
                new ShootFromDistanceToHubCommandAuto(shooter, indexer, drive, testTurret, 5).withName("Shoot"));

        NamedCommands.registerCommand(
                "Stop Shooter",
                Commands.runOnce(
                                () -> {
                                    shooter.stopFlywheel();
                                    // intake.stopRoller();
                                    indexer.stopSpindexer();
                                    indexer.stopShooterIndexer();
                                },
                                shooter,
                                intake,
                                indexer)
                        .withName("Stop Shooter"));
    }

    private Command buildAllianceCorrectFarMidAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile =
                                    alliance == Alliance.Red ? NEAR_MID_SCORE_AUTO_FILE : FAR_MID_SCORE_AUTO_FILE;

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Far Mid Semantic Auto");
    }

    private Command buildAllianceCorrect2BlueFarMidAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "2Far Mid Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Far Mid Semantic Auto");
    }

    private Command buildAllianceCorrect2RedFarMidAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "2Near Mid Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Far Mid Semantic Auto");
    }

    private Command buildAllianceCorrectNearMidAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile =
                                    alliance == Alliance.Red ? FAR_MID_SCORE_AUTO_FILE : NEAR_MID_SCORE_AUTO_FILE;

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Near Mid Semantic Auto");
    }

    private Command buildAllianceCorrect2RedNearMidAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "2Far Mid Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Near Mid Semantic Auto");
    }

    private Command buildAllianceCorrect2BlueNearMidAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "2Near Mid Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Near Mid Semantic Auto");
    }

    private Command buildDepoAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Blue Depo Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Depo Auto");
    }

    private Command buildBlueNearSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Near Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Near Sweep Score Auto");
    }

    private Command buildBlueFarSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Far Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Far Sweep Score Auto");
    }

    private Command buildRedNearSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Far Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Far Sweep Score Auto");
    }

    private Command buildRedFarSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Near Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Far Sweep Score Auto");
    }

    private Command buildFastRedFarSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Fast Near Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("FastFar Sweep Score Auto");
    }

    private Command buildFastRedNearSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Fast Far Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Fast Near Sweep Score Auto");
    }

    private Command buildFastBlueFarSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Fast Far Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Fast Far Sweep Score Auto");
    }

    private Command buildFastBlueNearSweepAuto() {
        return Commands.defer(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            String autoFile = "Fast Near Sweep Score Auto";

                            PathPlannerAuto auto = new PathPlannerAuto(autoFile);
                            drive.resetOdometry(auto.getStartingPose());
                            // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());

                            return auto;
                        },
                        java.util.Set.of(drive, intake, shooter, indexer))
                .withName("Fast Near Sweep Score Auto");
    }

    private Command buildDriveForwardAuto() {
        PathPlannerAuto auto = new PathPlannerAuto(DRIVE_FORWARD_AUTO_FILE);
        drive.resetOdometry(auto.getStartingPose());
        // Logger.recordOutput("Auto/StartingPose", auto.getStartingPose());
        return auto;
    }

    private Command buildShootOnlyAutoNearBlue() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Pose2d startingPose;
        startingPose = new Pose2d(3.533, 0.598, new Rotation2d(Math.PI / 2));

        commandGroup.addCommands(
                Commands.runOnce(() -> drive.resetOdometry(startingPose)),
                new ShootFromDistanceToHubCommandAuto(shooter, indexer, drive, testTurret, 5));
        return commandGroup;
    }

    private Command buildShootOnlyAutoNearRed() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Pose2d startingPose;
        startingPose = new Pose2d(12.967, 0.598, new Rotation2d(-Math.PI / 2));
        commandGroup.addCommands(
                Commands.runOnce(() -> drive.resetOdometry(startingPose)),
                new ShootFromDistanceToHubCommandAuto(shooter, indexer, drive, testTurret, 5));
        return commandGroup;
    }

    private Command buildShootOnlyAutoFarBlue() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Pose2d startingPose;
        startingPose = new Pose2d(3.533, 7.407, new Rotation2d(Math.PI / 2));
        commandGroup.addCommands(
                Commands.runOnce(() -> drive.resetOdometry(startingPose)),
                new ShootFromDistanceToHubCommandAuto(shooter, indexer, drive, testTurret, 5));
        return commandGroup;
    }

    private Command buildShootOnlyAutoFarRed() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Pose2d startingPose;
        startingPose = new Pose2d(12.967, 7.407, new Rotation2d(Math.PI));

        commandGroup.addCommands(
                Commands.runOnce(() -> drive.resetOdometry(startingPose)),
                new ShootFromDistanceToHubCommandAuto(shooter, indexer, drive, testTurret, 5));
        return commandGroup;
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
        // testTurret.setDefaultCommand(new TurretFollowOdometryCommand(vision, testTurret, drive, 0));

        controller.b().whileTrue(new TurretFollowCommand(vision, testTurret, drive, 0));
        navigator.povUp().onTrue(new TestTurretToPosCommand(testTurret, -90));
        navigator.povDown().onTrue(new TestTurretToPosCommand(testTurret, 90));
        navigator.povLeft().onTrue(new TestTurretToPosCommand(testTurret, 0));
        navigator.povRight().onTrue(new TestTurretToPosCommand(testTurret, 180));

        navigator.b().onTrue(Commands.runOnce(() -> climb.setDutyCycle(0.4)));
        navigator.a().onTrue(Commands.runOnce(() -> climb.setDutyCycle(-0.4)));

        controller.povDown().whileTrue(new OutakeBallsCommand(intake));

        controller
                .start()
                .onTrue(Commands.runOnce(() ->
                                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())))
                        .ignoringDisable(true));
        controller
                .leftTrigger()
                .whileTrue(DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY() * 0.65,
                        () -> -controller.getLeftX() * 0.65,
                        () -> -controller.getRightX() * 0.65));
        controller.rightTrigger().whileTrue(new IntakeBallsCommand(intake));

        controller.a().whileTrue(new IntakePulseCommand(intake));
        // controller
        //         .b()
        //         .whileTrue(new ShootFromDistanceCommand(
        //                 shooter,
        //                 () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
        //                         Constants.ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_KEY,
        //                         Constants.ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT)));
        controller.x().whileTrue(new ShootCommand(shooter));
        controller
                .b()
                .whileTrue(new ShuttleShot(
                        shooter, () -> drive.getPose().getTranslation().getY(), testTurret, drive));

        // ParallelCommandGroup shootGroup = new ParallelCommandGroup(
        //         new ShootFromDistanceToHubCommand(shooter),
        //         new TurretFollowOdometryCommand(vision, testTurret, drive, 0));
        controller.leftBumper().whileTrue(new ShootFromDistanceToHubCommand(shooter, testTurret, drive));
        controller
                .y()
                .whileTrue(Commands.startEnd(
                        () -> indexer.setSpindexerDutyCycle(-0.5), () -> indexer.stopSpindexer(), indexer));
        controller.povUp().whileTrue(new ZeroTurretCommand(testTurret));
        controller
                .rightBumper()
                .and(controller.leftBumper())
                .whileTrue(new RunIndexterDutyCycle(
                        indexer,
                        Constants.ShooterConstants.SPINDEXER_FEED_DUTY,
                        Constants.ShooterConstants.SHOOTER_KICKER_FEED_DUTY));
        controller
                .rightBumper()
                .and(controller.b())
                .whileTrue(new RunIndexterDutyCycle(
                        indexer,
                        Constants.ShooterConstants.SPINDEXER_FEED_DUTY,
                        Constants.ShooterConstants.SHOOTER_KICKER_FEED_DUTY));
        // controller
        //         .rightBumper()
        //         .and(controller.x())
        //         .whileTrue(new TestTurretToPosCommandStatic(testTurret)
        //                 .andThen(new RunIndexterDutyCycle(
        //                         indexer,
        //                         Constants.ShooterConstants.SPINDEXER_FEED_DUTY,
        //                         Constants.ShooterConstants.SHOOTER_KICKER_FEED_DUTY)));
        controller
                .rightBumper()
                .and(controller.x())
                .whileTrue(new RunIndexterDutyCycle(
                        indexer,
                        Constants.ShooterConstants.SPINDEXER_FEED_DUTY,
                        Constants.ShooterConstants.SHOOTER_KICKER_FEED_DUTY));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
        // return Commands.runOnce(() -> shooter.runShot(0.5, 55), shooter)
        //         .andThen(Commands.runOnce(() -> testTurret.setTargetAngleRadians(Math.toRadians(-24))))
        //         .andThen(Commands.runOnce(() -> indexer.index(0.5), indexer))
        //         .until(() -> false);
    }

    public void resetSimulation() {
        // TODO see why this is running later
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        field.setRobotPose(driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    public Pose2d getSimulationPose() {
        if (driveSimulation == null) {
            return drive.getPose();
        }
        return driveSimulation.getSimulatedDriveTrainPose();
    }

    public Pose2d getDrivePose() {
        return drive.getPose();
    }

    public void updateDashboardField() {
        Pose2d robotPose = drive.getPose();
        field.setRobotPose(robotPose);

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d targetPose = TurretGridSelector.select(robotPose, alliance, java.util.Optional.empty())
                .targetPose()
                .toPose2d();
        field.getObject("TurretTarget").setPose(targetPose);
    }

    public int getGridSelectorTagID() {
        Pose2d robotPose = drive.getPose();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        GridTarget target = TurretGridSelector.select(robotPose, alliance, Optional.empty());

        return target.primaryTagId();
    }

    public String getGridZone() {
        Pose2d robotPose = drive.getPose();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        GridTarget target = TurretGridSelector.select(robotPose, alliance, Optional.empty());
        return target.zone().name();
    }

    public double calculateDistanceToTargetMeters() {

        return cam0VisionIO.calcTagDistance();
    }

    public double calculateDistanceToHubMeters() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double hubX, hubY;
        if (alliance == Alliance.Blue) {
            hubX = Constants.blueHubX;
            hubY = Constants.blueHubY;
        } else {
            hubX = Constants.redHubX;
            hubY = Constants.redHubY;
        }

        return drive.distanceToTargetMeters(new Translation2d(hubX, hubY));
    }

    public double getRobotY() {
        return drive.getPose().getTranslation().getY();
    }

    public void stopShooter() {
        shooter.runFlywheelDutyCycle(0);
        indexer.setSpindexerDutyCycle(0);
        indexer.setShooterIndexerDutyCycle(0);
    }

    public LEDs getLEDs() {
        return leds;
    }

    public void ledsOn() {
        leds.setRGB(255, 255, 255);
    }
}

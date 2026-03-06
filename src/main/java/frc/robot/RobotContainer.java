package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.IntakeStowCommand;
import frc.robot.commands.RunIndexterDutyCycle;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TestTurretFollowCommand;
import frc.robot.commands.TurretFlipCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final Vision vision;
    private final Drive drive;
    private final Shooter shooter;
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private SwerveDriveSimulation driveSimulation = null;

    private final CommandXboxController controller = new CommandXboxController(0);

    private final TestTurretSubsystem testTurret = new TestTurretSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final LoggedDashboardChooser<Command> autoChooser;

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
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                        new VisionIOPhotonVision(camera1Name, robotToCamera1));
                shooter = new Shooter(new ShooterIOTalonFX());
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
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                shooter = new Shooter(new ShooterIOSim());
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
                shooter = new Shooter(new ShooterIO() {});
                break;
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        controller
                .start()
                .onTrue(Commands.runOnce(() ->
                                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())))
                        .ignoringDisable(true));
        controller
                .leftTrigger()
                .whileTrue(DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY() * 0.5,
                        () -> -controller.getLeftX() * 0.5,
                        () -> -controller.getRightX() * 0.5));
        controller.rightTrigger().whileTrue(new IntakeOutCommand(intake));

        controller.a().onTrue(new IntakeStowCommand(intake));

        Command followCommand = new TestTurretFollowCommand(testTurret, vision, drive::getPose);
        Command flipCommand = new TurretFlipCommand(testTurret);

        Command followFlipCommand = Commands.repeatingSequence(
                followCommand.until(testTurret::isAtLimit), flipCommand, Commands.waitSeconds(1));

        controller.y().whileTrue(followFlipCommand);

        controller.leftBumper().whileTrue(new ShootCommand(shooter));

        controller
                .rightBumper()
                .and(controller.leftBumper())
                .whileTrue(new RunIndexterDutyCycle(
                        indexer,
                        Constants.ShooterConstants.SPINDEXER_FEED_DUTY,
                        Constants.ShooterConstants.SHOOTER_KICKER_FEED_DUTY));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
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
}

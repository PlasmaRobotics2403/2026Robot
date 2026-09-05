package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    static final double ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 2.255;
    public static final double SIM_BUMPER_LENGTH_IN = 32.5;
    public static final double SIM_BUMPER_WIDTH_IN = 32.5;

    public double velocityToMetersConstant = -1;

    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    private static DriveTrainSimulationConfig mapleSimConfig = null;

    public static DriveTrainSimulationConfig getMapleSimConfig() {
        if (mapleSimConfig != null) return mapleSimConfig;

        return mapleSimConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                .withCustomModuleTranslations(getModuleTranslations())
                .withBumperSize(Inches.of(SIM_BUMPER_LENGTH_IN), Inches.of(SIM_BUMPER_WIDTH_IN))
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getFalcon500(1),
                        TunerConstants.FrontLeft.DriveMotorGearRatio,
                        TunerConstants.FrontLeft.SteerMotorGearRatio,
                        Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                        Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                        Inches.of(2),
                        KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                        WHEEL_COF));
    }

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        PhoenixOdometryThread.getInstance().start();

        AutoBuilder.configureCustom(
                path -> new ReplanningPathCommand(
                        path,
                        this::getPose,
                        this::getChassisSpeeds,
                        this::runVelocity,
                        Drive::createPathController,
                        PP_CONFIG,
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                        this),
                this::getPose,
                this::resetOdometry,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                true);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        SmartDashboard.putNumber("VelocityToMetersConstant", velocityToMetersConstant);
    }

    private static PPHolonomicDriveController createPathController() {
        return new PPHolonomicDriveController(
                new PIDConstants(
                        DriveConstants.PATHPLANNER_TRANSLATION_KP,
                        DriveConstants.PATHPLANNER_TRANSLATION_KI,
                        DriveConstants.PATHPLANNER_TRANSLATION_KD),
                new PIDConstants(
                        DriveConstants.PATHPLANNER_ROTATION_KP,
                        DriveConstants.PATHPLANNER_ROTATION_KI,
                        DriveConstants.PATHPLANNER_ROTATION_KD));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        Logger.recordOutput("Swerve/Estimated Pos", getPose());

        velocityToMetersConstant = SmartDashboard.getNumber("VelocityToMetersConstant", -1);

        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    /** @param speeds Speeds in meters/sec */
    public void runVelocity(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public Translation2d getVelocityCompensatedTargetTranslation(Translation2d targetField) {
        ChassisSpeeds fieldRelativeSpeeds = getFieldRelativeVelocity();

        double offsetX = fieldRelativeSpeeds.vxMetersPerSecond * velocityToMetersConstant;
        double offsetY = fieldRelativeSpeeds.vyMetersPerSecond * velocityToMetersConstant;

        return new Translation2d(targetField.getX() + offsetX, targetField.getY() + offsetY);
        // return new Translation2d(targetField.getX(), targetField.getY());
    }

    public double calcTurretAngle(Translation2d targetField) {
        Pose2d robotPose = getPose();
        // robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d());
        // Rotation2d robotHeading = robotPose.getRotation();
        Translation2d robotTranslation = robotPose.getTranslation();

        Translation2d compensatedTarget = getVelocityCompensatedTargetTranslation(targetField);

        Translation2d turretOffsetRobot = TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER;
        Translation2d turretOffsetField = turretOffsetRobot.rotateBy(
                new Rotation2d(robotPose.getRotation().getRadians()));
        Translation2d turretField = robotTranslation.plus(turretOffsetField);

        // Translation2d turretToTargetField = compensatedTarget.minus(robotTranslation);
        Translation2d turretToTargetField = compensatedTarget.minus(turretField);

        Rotation2d targetFieldAngle = turretToTargetField.getAngle();
        Rotation2d targetRobotAngle =
                targetFieldAngle.minus(new Rotation2d(robotPose.getRotation().getRadians()));

        // Turret zero is robot-left
        Rotation2d turretZeroAngle = Rotation2d.fromDegrees(90.0);
        Rotation2d turretCommand = turretZeroAngle.minus(targetRobotAngle);

        double targetRadians = MathUtil.angleModulus(turretCommand.getRadians());
        targetRadians = applyTurretUnwind(targetRadians);

        return targetRadians;
    }

    public double calcTurretAngleAuto(Translation2d targetField) {
        Pose2d robotPose = getPose();
        // robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d());
        // Rotation2d robotHeading = robotPose.getRotation();
        Translation2d robotTranslation = robotPose.getTranslation();

        Translation2d compensatedTarget = getVelocityCompensatedTargetTranslation(targetField);

        Translation2d turretOffsetRobot = TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER;
        Translation2d turretOffsetField = turretOffsetRobot.rotateBy(
                new Rotation2d(robotPose.getRotation().getRadians()));
        Translation2d turretField = robotTranslation.plus(turretOffsetField);

        // Translation2d turretToTargetField = compensatedTarget.minus(robotTranslation);
        Translation2d turretToTargetField = compensatedTarget.minus(turretField);

        Rotation2d targetFieldAngle = turretToTargetField.getAngle();
        Rotation2d targetRobotAngle =
                targetFieldAngle.minus(new Rotation2d(robotPose.getRotation().getRadians()));

        // Turret zero is robot-left
        Rotation2d turretZeroAngle = Rotation2d.fromDegrees(90.0);
        Rotation2d turretCommand = turretZeroAngle.minus(targetRobotAngle);

        double targetRadians = MathUtil.angleModulus(turretCommand.getRadians());
        targetRadians = applyTurretUnwind(targetRadians);

        return targetRadians;
    }

    private double applyTurretUnwind(double targetAngle) {
        double min = Math.toRadians(TurretConstants.MIN_ANGLE_DEG);
        double max = Math.toRadians(TurretConstants.MAX_ANGLE_DEG);

        while (targetAngle < min - Math.toRadians(3.0)) {
            targetAngle += 2.0 * Math.PI;
        }

        while (targetAngle > max + Math.toRadians(3.0)) {
            targetAngle -= 2.0 * Math.PI;
        }

        return targetAngle;
    }

    public double distanceToTargetMeters(Translation2d targetField) {
        Translation2d compensatedTarget = getVelocityCompensatedTargetTranslation(targetField);

        return getPose().getTranslation().getDistance(compensatedTarget);
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotRelativeVelocity() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        ChassisSpeeds robot = getRobotRelativeVelocity();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robot, getRotation());
    }

    public void resetOdometry(Pose2d pose) {
        DriverStation.reportWarning("reset swerve heading", false);
        resetSimulationPoseCallBack.accept(pose);
        // pose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.k180deg);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
}

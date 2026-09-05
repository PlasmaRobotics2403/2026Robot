package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TurretGridSelector;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import frc.robot.util.TurretTargetingUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TestTurretFollowCommand extends Command {
    private static final String kFollowOffsetDegKey = "Turret/Follow/OffsetDeg";
    private static final String kUseFeedPointKey = "Turret/Follow/UseFeedPoint";

    private final TestTurretSubsystem turret;
    private final Vision vision;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final int cameraIndex;
    private Optional<GridZone> previousZone = Optional.empty();
    private final Debouncer tagLockEnter = new Debouncer(TurretConstants.TAG_LOCK_ENTER_DEBOUNCE_SEC);
    private final Debouncer tagLockExit = new Debouncer(TurretConstants.TAG_LOCK_EXIT_DEBOUNCE_SEC);
    private boolean cameraLockActive = false;
    private Rotation2d lastYaw = new Rotation2d();

    public TestTurretFollowCommand(
            TestTurretSubsystem turret, Vision vision, Supplier<Pose2d> robotPoseSupplier, int cameraIndex) {
        this.turret = turret;
        this.vision = vision;
        this.robotPoseSupplier = robotPoseSupplier;
        this.cameraIndex = cameraIndex;
        addRequirements(turret);
    }

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision, Supplier<Pose2d> robotPoseSupplier) {
        this(turret, vision, robotPoseSupplier, 0);
    }

    @Override
    public void initialize() {
        previousZone = Optional.empty();
        turret.clearUnwinding();
        turret.setTargetAngleRadians(turret.getPositionRadians());
        SmartDashboard.putNumber(kFollowOffsetDegKey, SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0));
        SmartDashboard.putBoolean(kUseFeedPointKey, SmartDashboard.getBoolean(kUseFeedPointKey, false));
        SmartDashboard.putNumber("Turret/Follow/TxDeg", 0.0);
        cameraLockActive = false;
        lastYaw = new Rotation2d();
        tagLockEnter.calculate(false);
        tagLockExit.calculate(true);
    }

    @Override
    public void execute() {
        double followOffsetDeg = SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0);
        double followOffsetRad = Units.degreesToRadians(followOffsetDeg);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d robotPose = robotPoseSupplier.get();
        boolean useFeedPoint = SmartDashboard.getBoolean(kUseFeedPointKey, false);
        TurretTargetingUtil.FollowSolution followSolution;
        if (useFeedPoint) {
            followSolution = TurretTargetingUtil.calculateFieldPointAimSolution(
                    robotPose, TurretConstants.DEFAULT_FEED_FIELD_POINT);
        } else {
            GridTarget gridTarget = TurretGridSelector.select(robotPose, alliance, previousZone);
            Optional<Rotation2d> rawYaw = vision.getTargetYaw(cameraIndex, gridTarget.trimTagIds());
            if (rawYaw.isPresent()) {
                lastYaw = rawYaw.get();
            }

            if (cameraLockActive) {
                cameraLockActive = tagLockExit.calculate(rawYaw.isPresent());
            } else {
                cameraLockActive = tagLockEnter.calculate(rawYaw.isPresent());
            }

            Optional<Rotation2d> yawForTarget = cameraLockActive ? Optional.of(lastYaw) : Optional.empty();
            followSolution = TurretTargetingUtil.calculateTagAimSolution(
                    robotPose, gridTarget, yawForTarget, turret.getPositionRadians(), followOffsetRad, followOffsetRad);
        }

        GridTarget gridTarget = followSolution.gridTarget();
        if (gridTarget != null) {
            previousZone = Optional.of(gridTarget.zone());
        }

        double cameraYawRad = followSolution.cameraYawRad();
        double requestedTargetAngleRad = MathUtil.angleModulus(followSolution.finalTargetAngleRad());

        TurretTargetingUtil.FlipPlan flipPlan = TurretTargetingUtil.planFlip(
                turret.getPositionRadians(), requestedTargetAngleRad, turret.MIN_ANGLE_RAD, turret.MAX_ANGLE_RAD);
        boolean unwindRequested = flipPlan.shouldUnwind();
        double preClampCommandAngleRad = unwindRequested
                ? flipPlan.unwindTargetAngleRad().orElse(requestedTargetAngleRad)
                : requestedTargetAngleRad;
        double commandedAngleRad = MathUtil.clamp(preClampCommandAngleRad, turret.MIN_ANGLE_RAD, turret.MAX_ANGLE_RAD);
        boolean saturated = Math.abs(preClampCommandAngleRad - commandedAngleRad) > 1e-6;

        turret.setTargetAngleRadians(commandedAngleRad);
        turret.updateUnwinding(unwindRequested);
        boolean unwindActive = turret.isUnwinding();

        Logger.recordOutput("Turret/Follow/OffsetDeg", followOffsetDeg);
        Logger.recordOutput("Turret/Follow/Alliance", alliance.toString());
        Logger.recordOutput(
                "Turret/Follow/Zone", gridTarget != null ? gridTarget.zone().toString() : "N/A");
        Logger.recordOutput(
                "Turret/Follow/TargetType", followSolution.targetType().toString());
        Logger.recordOutput("Turret/Follow/PrimaryTagId", gridTarget != null ? gridTarget.primaryTagId() : -1);
        Logger.recordOutput("Turret/Follow/TrimTagIds", gridTarget != null ? gridTarget.trimTagIds() : new int[0]);
        Logger.recordOutput(
                "Turret/Follow/TargetPointX", followSolution.targetPoint().getX());
        Logger.recordOutput(
                "Turret/Follow/TargetPointY", followSolution.targetPoint().getY());
        Logger.recordOutput(
                "Turret/Follow/PoseAimDeg", followSolution.poseAimAngle().getDegrees());
        Logger.recordOutput(
                "Turret/Follow/TargetYawSeen", followSolution.yawForTarget().isPresent());
        Logger.recordOutput(
                "Turret/Follow/UsingCameraAim",
                followSolution.aimMode() == TurretTargetingUtil.AimMode.TAG_CAMERA_LOCK);
        Logger.recordOutput(
                "Turret/Follow/UsingOdometryAim",
                followSolution.aimMode() == TurretTargetingUtil.AimMode.TAG_ODOMETRY_FALLBACK);
        Logger.recordOutput("Turret/Follow/CameraLockActive", cameraLockActive);
        Logger.recordOutput("Turret/Follow/UsingSpecificTag", gridTarget != null);
        Logger.recordOutput("Turret/Follow/TxDeg", Units.radiansToDegrees(cameraYawRad));
        Logger.recordOutput("Turret/Follow/CameraYawDeg", Units.radiansToDegrees(cameraYawRad));
        Logger.recordOutput("Turret/Follow/RequestedTargetDeg", Units.radiansToDegrees(requestedTargetAngleRad));
        Logger.recordOutput("Turret/Follow/PreClampCommandedAngleDeg", Units.radiansToDegrees(preClampCommandAngleRad));
        Logger.recordOutput("Turret/Follow/CommandedAngleDeg", Units.radiansToDegrees(commandedAngleRad));
        Logger.recordOutput("Turret/Follow/UnwindRequested", unwindRequested);
        Logger.recordOutput("Turret/Follow/UnwindActive", unwindActive);
        Logger.recordOutput(
                "Turret/Follow/UnwindTargetDeg",
                flipPlan.unwindTargetAngleRad().isPresent()
                        ? Units.radiansToDegrees(flipPlan.unwindTargetAngleRad().getAsDouble())
                        : Double.NaN);
        Logger.recordOutput("Turret/Follow/Saturated", saturated);
        Logger.recordOutput("Turret/Follow/AtLimit", turret.isAtLimit());

        SmartDashboard.putString("Turret/Follow/Alliance", alliance.toString());
        SmartDashboard.putString(
                "Turret/Follow/Zone", gridTarget != null ? gridTarget.zone().toString() : "N/A");
        SmartDashboard.putString(
                "Turret/Follow/TargetType", followSolution.targetType().toString());
        SmartDashboard.putNumber("Turret/Follow/PrimaryTagId", gridTarget != null ? gridTarget.primaryTagId() : -1);
        SmartDashboard.putNumber(
                "Turret/Follow/TargetPointX", followSolution.targetPoint().getX());
        SmartDashboard.putNumber(
                "Turret/Follow/TargetPointY", followSolution.targetPoint().getY());
        SmartDashboard.putNumber("Turret/Follow/TxDeg", Units.radiansToDegrees(cameraYawRad));
        SmartDashboard.putBoolean(
                "Turret/Follow/TargetYawSeen", followSolution.yawForTarget().isPresent());
        SmartDashboard.putBoolean(
                "Turret/Follow/UsingCameraAim",
                followSolution.aimMode() == TurretTargetingUtil.AimMode.TAG_CAMERA_LOCK);
        SmartDashboard.putBoolean(
                "Turret/Follow/UsingOdometryAim",
                followSolution.aimMode() == TurretTargetingUtil.AimMode.TAG_ODOMETRY_FALLBACK);
        SmartDashboard.putBoolean("Turret/Follow/CameraLockActive", cameraLockActive);
        SmartDashboard.putBoolean("Turret/Follow/UnwindActive", unwindActive);
        SmartDashboard.putBoolean("Turret/Follow/Saturated", saturated);
        SmartDashboard.putBoolean("Turret/Follow/AtLimit", turret.isAtLimit());
        SmartDashboard.putNumber("Turret/Follow/RequestedTargetDeg", Units.radiansToDegrees(requestedTargetAngleRad));
        SmartDashboard.putNumber("Turret/Follow/CommandedAngleDeg", Units.radiansToDegrees(commandedAngleRad));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

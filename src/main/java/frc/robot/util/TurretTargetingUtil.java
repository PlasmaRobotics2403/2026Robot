package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import java.util.Optional;
import java.util.OptionalDouble;

public final class TurretTargetingUtil {
    private static final double FLIP_LIMIT_MARGIN_RAD = Math.toRadians(25.0);
    private static final double UNWIND_TRIGGER_MARGIN_RAD = Math.toRadians(18.0);
    private static final double MIN_UNWIND_MOVE_RAD = Math.toRadians(2.0);
    private static final InterpolatingDoubleTreeMap FEED_ANGLE_TABLE_DEG = buildFeedAngleTableDeg();

    private TurretTargetingUtil() {}

    private static InterpolatingDoubleTreeMap buildFeedAngleTableDeg() {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        table.put(1.0, -10.0);
        table.put(3.0, -20.0);
        table.put(5.0, -30.0);
        return table;
    }

    public static Translation2d getTurretPivotTranslation(Pose2d robotPose) {
        Transform2d robotToPivot = new Transform2d(TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER, Rotation2d.kZero);
        return robotPose.transformBy(robotToPivot).getTranslation();
    }

    public static Pose3d getTurretCameraPose(Pose2d robotPose, double turretAngleRad) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d turretPivotPose = robotPose3d.transformBy(new Transform3d(
                TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER.getX(),
                TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER.getY(),
                0.0,
                new edu.wpi.first.math.geometry.Rotation3d()));
        Transform3d turretRotation =
                new Transform3d(0.0, 0.0, 0.0, new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, turretAngleRad));
        return turretPivotPose.transformBy(turretRotation).transformBy(TurretConstants.TURRET_TO_CAMERA);
    }

    public static FollowSolution calculateTagAimSolution(
            Pose2d robotPose,
            GridTarget gridTarget,
            Optional<Rotation2d> yawForTarget,
            double currentTurretAngleRad,
            double odometryOffsetRad,
            double cameraOffsetRad) {
        Pose2d targetPose2d = gridTarget.targetPose().toPose2d();

        Rotation2d poseAimAngle = calculatePoseAimFromTarget(robotPose, targetPose2d);
        double odometryTargetAngleRad = toTurretNativeAngleRad(poseAimAngle.getRadians(), odometryOffsetRad);
        double cameraYawRad = yawForTarget.map(Rotation2d::getRadians).orElse(0.0);
        double cameraTargetAngleRad = MathUtil.angleModulus(currentTurretAngleRad + cameraYawRad + cameraOffsetRad);

        boolean cameraLockActive = yawForTarget.isPresent();
        AimMode aimMode = cameraLockActive ? AimMode.TAG_CAMERA_LOCK : AimMode.TAG_ODOMETRY_FALLBACK;
        double finalTargetAngleRad = cameraLockActive ? cameraTargetAngleRad : odometryTargetAngleRad;

        return new FollowSolution(
                aimMode,
                TargetType.TAG,
                gridTarget,
                targetPose2d.getTranslation(),
                poseAimAngle,
                yawForTarget,
                cameraYawRad,
                odometryTargetAngleRad,
                cameraTargetAngleRad,
                finalTargetAngleRad,
                cameraLockActive);
    }

    public static FollowSolution calculateTagAimSolution(
            Pose2d robotPose,
            Alliance alliance,
            Optional<GridZone> previousZone,
            Optional<Rotation2d> yawForTarget,
            double currentTurretAngleRad,
            double followOffsetRad) {
        GridTarget gridTarget = TurretGridSelector.select(robotPose, alliance, previousZone);
        return calculateTagAimSolution(
                robotPose, gridTarget, yawForTarget, currentTurretAngleRad, followOffsetRad, followOffsetRad);
    }

    public static FollowSolution calculateFollowSolution(
            Pose2d robotPose,
            Alliance alliance,
            Optional<GridZone> previousZone,
            Vision vision,
            int cameraIndex,
            double currentTurretAngleRad,
            double odometryOffsetRad,
            double cameraOffsetRad) {
        GridTarget gridTarget = TurretGridSelector.select(robotPose, alliance, previousZone);
        Optional<Rotation2d> yawForTarget = vision.getTargetYaw(cameraIndex, gridTarget.trimTagIds());
        return calculateTagAimSolution(
                robotPose, gridTarget, yawForTarget, currentTurretAngleRad, odometryOffsetRad, cameraOffsetRad);
    }

    public static FollowSolution calculateFollowSolution(
            Pose2d robotPose,
            Alliance alliance,
            Optional<GridZone> previousZone,
            Vision vision,
            int cameraIndex,
            double currentTurretAngleRad,
            double followOffsetRad) {
        return calculateFollowSolution(
                robotPose,
                alliance,
                previousZone,
                vision,
                cameraIndex,
                currentTurretAngleRad,
                followOffsetRad,
                followOffsetRad);
    }

    public static FollowSolution calculateFieldPointAimSolution(Pose2d robotPose, Translation2d fieldTargetPoint) {
        Rotation2d poseAimAngle = calculatePoseAimFromTarget(robotPose, fieldTargetPoint);
        double distanceMeters = getTurretPivotTranslation(robotPose).getDistance(fieldTargetPoint);
        double interpolatedAngleDeg = FEED_ANGLE_TABLE_DEG.get(distanceMeters);
        double interpolatedAngleRad = Units.degreesToRadians(interpolatedAngleDeg);
        return new FollowSolution(
                AimMode.FIELD_POINT_INTERPOLATED,
                TargetType.FIELD_POINT,
                null,
                fieldTargetPoint,
                poseAimAngle,
                Optional.empty(),
                0.0,
                interpolatedAngleRad,
                interpolatedAngleRad,
                interpolatedAngleRad,
                false);
    }

    private static Rotation2d calculatePoseAimFromTarget(Pose2d robotPose, Pose2d targetPose) {
        return calculatePoseAimFromTarget(robotPose, targetPose.getTranslation());
    }

    private static Rotation2d calculatePoseAimFromTarget(Pose2d robotPose, Translation2d targetTranslation) {
        Translation2d turretPivot = getTurretPivotTranslation(robotPose);
        Rotation2d fieldToAimPoint = new Rotation2d(
                targetTranslation.getX() - turretPivot.getX(), targetTranslation.getY() - turretPivot.getY());
        return fieldToAimPoint.minus(robotPose.getRotation());
    }

    private static double toTurretNativeAngleRad(double thetaRobotRelativeRad, double followOffsetRad) {
        return MathUtil.angleModulus(Math.PI / 2.0 - thetaRobotRelativeRad + followOffsetRad);
    }

    public static FlipPlan planFlip(
            double currentAngleRad, double gridFollowTargetAngleRad, double minAngleRad, double maxAngleRad) {
        double positiveLimitTargetRad = MathUtil.clamp(maxAngleRad - FLIP_LIMIT_MARGIN_RAD, minAngleRad, maxAngleRad);
        double negativeLimitTargetRad = MathUtil.clamp(minAngleRad + FLIP_LIMIT_MARGIN_RAD, minAngleRad, maxAngleRad);

        FlipBranch chosenBranch = currentAngleRad >= 0.0 ? FlipBranch.NEGATIVE_LIMIT : FlipBranch.POSITIVE_LIMIT;
        double requestedFlipAngleRad =
                chosenBranch == FlipBranch.POSITIVE_LIMIT ? positiveLimitTargetRad : negativeLimitTargetRad;

        boolean currentOnPositiveSide = currentAngleRad >= 0.0;
        double distanceToCurrentLimitRad =
                currentOnPositiveSide ? maxAngleRad - currentAngleRad : currentAngleRad - minAngleRad;
        double currentSideLimitTargetRad = currentOnPositiveSide ? positiveLimitTargetRad : negativeLimitTargetRad;
        boolean gridTargetPushesTowardCurrentLimit = currentOnPositiveSide
                ? gridFollowTargetAngleRad > currentAngleRad
                : gridFollowTargetAngleRad < currentAngleRad;
        boolean shouldUnwind = distanceToCurrentLimitRad <= UNWIND_TRIGGER_MARGIN_RAD
                && gridTargetPushesTowardCurrentLimit
                && Math.abs(currentSideLimitTargetRad - currentAngleRad) > MIN_UNWIND_MOVE_RAD;

        return new FlipPlan(
                requestedFlipAngleRad,
                chosenBranch,
                shouldUnwind,
                shouldUnwind ? OptionalDouble.of(currentSideLimitTargetRad) : OptionalDouble.empty(),
                requestedFlipAngleRad);
    }

    public enum FlipBranch {
        POSITIVE_LIMIT,
        NEGATIVE_LIMIT
    }

    public enum AimMode {
        TAG_ODOMETRY_FALLBACK,
        TAG_CAMERA_LOCK,
        FIELD_POINT_INTERPOLATED
    }

    public enum TargetType {
        TAG,
        FIELD_POINT
    }

    public record FollowSolution(
            AimMode aimMode,
            TargetType targetType,
            GridTarget gridTarget,
            Translation2d targetPoint,
            Rotation2d poseAimAngle,
            Optional<Rotation2d> yawForTarget,
            double cameraYawRad,
            double odometryTargetAngleRad,
            double cameraTargetAngleRad,
            double finalTargetAngleRad,
            boolean cameraLockActive) {}

    public record FlipPlan(
            double requestedFlipAngleRad,
            FlipBranch chosenBranch,
            boolean shouldUnwind,
            OptionalDouble unwindTargetAngleRad,
            double finalTargetAngleRad) {}
}

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import java.util.Optional;
import java.util.OptionalDouble;

public final class TurretTargetingUtil {
    private static final double FLIP_LIMIT_MARGIN_RAD = Math.toRadians(25.0);
    private static final double UNWIND_TRIGGER_MARGIN_RAD = Math.toRadians(18.0);
    private static final double MIN_UNWIND_MOVE_RAD = Math.toRadians(2.0);

    private TurretTargetingUtil() {}

    public static FollowSolution calculateFollowSolution(
            Pose2d robotPose,
            Alliance alliance,
            Optional<GridZone> previousZone,
            Vision vision,
            int cameraIndex,
            double followOffsetRad) {
        GridTarget gridTarget = TurretGridSelector.select(robotPose, alliance, previousZone);
        Pose2d targetPose2d = gridTarget.targetPose().toPose2d();

        Rotation2d fieldToAimPoint =
                new Rotation2d(targetPose2d.getX() - robotPose.getX(), targetPose2d.getY() - robotPose.getY());
        Rotation2d poseAimAngle = fieldToAimPoint.minus(robotPose.getRotation());

        Optional<Rotation2d> txForTarget = vision.getTargetX(cameraIndex, gridTarget.trimTagIds());
        double txTrimRad = txForTarget.map(Rotation2d::getRadians).orElse(0.0);
        double targetAngleRad = MathUtil.angleModulus(poseAimAngle.getRadians() + txTrimRad - followOffsetRad);

        return new FollowSolution(gridTarget, poseAimAngle, txForTarget, txTrimRad, targetAngleRad);
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

    public record FollowSolution(
            GridTarget gridTarget,
            Rotation2d poseAimAngle,
            Optional<Rotation2d> txForTarget,
            double txTrimRad,
            double targetAngleRad) {}

    public record FlipPlan(
            double requestedFlipAngleRad,
            FlipBranch chosenBranch,
            boolean shouldUnwind,
            OptionalDouble unwindTargetAngleRad,
            double finalTargetAngleRad) {}
}

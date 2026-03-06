package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Map;
import java.util.Optional;

public final class TurretGridSelector {
    private static final double ZONE_HYSTERESIS_METERS = 0.10;
    private static final double INBOARD_POINT_OFFSET_METERS = 1.00;

    private static final AprilTagFieldLayout FIELD_LAYOUT = VisionConstants.aprilTagLayout;
    private static final double FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
    private static final double FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();
    private static final double FIELD_CENTER_X_METERS = FIELD_LENGTH_METERS / 2.0;
    private static final double FIELD_CENTER_Y_METERS = FIELD_WIDTH_METERS / 2.0;

    private static final double BLUE_GRID_CENTER_X_METERS = 4.5366686;
    private static final double RED_GRID_CENTER_X_METERS = 12.0043702;

    private static final OuterWedgeMapping BLUE_MAPPING = new OuterWedgeMapping(
            new Pose2d(BLUE_GRID_CENTER_X_METERS, 4.0346376, edu.wpi.first.math.geometry.Rotation2d.kZero),
            Map.of(
                    GridZone.BLUE_OUTER_TOP, createTagTarget(GridZone.BLUE_OUTER_TOP, 18, new int[] {18, 27}),
                    GridZone.BLUE_OUTER_CENTER, createTagTarget(GridZone.BLUE_OUTER_CENTER, 26, new int[] {25, 26}),
                    GridZone.BLUE_OUTER_BOTTOM, createTagTarget(GridZone.BLUE_OUTER_BOTTOM, 22, new int[] {22, 23})));

    private static final OuterWedgeMapping RED_MAPPING = new OuterWedgeMapping(
            new Pose2d(RED_GRID_CENTER_X_METERS, 4.0346376, edu.wpi.first.math.geometry.Rotation2d.kZero),
            Map.of(
                    GridZone.RED_OUTER_TOP, createTagTarget(GridZone.RED_OUTER_TOP, 5, new int[] {5, 8}),
                    GridZone.RED_OUTER_CENTER, createTagTarget(GridZone.RED_OUTER_CENTER, 10, new int[] {9, 10}),
                    GridZone.RED_OUTER_BOTTOM, createTagTarget(GridZone.RED_OUTER_BOTTOM, 2, new int[] {2, 11})));

    private static final GridTarget RED_MIDDLE_TOP_TARGET = createPointTarget(
            GridZone.RED_MIDDLE_TOP,
            new Translation2d(FIELD_LENGTH_METERS - INBOARD_POINT_OFFSET_METERS, INBOARD_POINT_OFFSET_METERS));
    private static final GridTarget RED_MIDDLE_BOTTOM_TARGET = createPointTarget(
            GridZone.RED_MIDDLE_BOTTOM,
            new Translation2d(
                    FIELD_LENGTH_METERS - INBOARD_POINT_OFFSET_METERS,
                    FIELD_WIDTH_METERS - INBOARD_POINT_OFFSET_METERS));
    private static final GridTarget BLUE_MIDDLE_TOP_TARGET = createPointTarget(
            GridZone.BLUE_MIDDLE_TOP, new Translation2d(INBOARD_POINT_OFFSET_METERS, INBOARD_POINT_OFFSET_METERS));
    private static final GridTarget BLUE_MIDDLE_BOTTOM_TARGET = createPointTarget(
            GridZone.BLUE_MIDDLE_BOTTOM,
            new Translation2d(INBOARD_POINT_OFFSET_METERS, FIELD_WIDTH_METERS - INBOARD_POINT_OFFSET_METERS));

    private TurretGridSelector() {}

    public static GridTarget select(Pose2d robotPose, Optional<GridZone> previousZone) {
        double x = robotPose.getX();
        double y = robotPose.getY();

        if (x <= BLUE_GRID_CENTER_X_METERS) {
            GridZone zone = selectOuterZone(robotPose, BLUE_MAPPING.gridCenter(), previousZone, false);
            return BLUE_MAPPING.targets().get(zone);
        }

        if (x >= RED_GRID_CENTER_X_METERS) {
            GridZone zone = selectOuterZone(robotPose, RED_MAPPING.gridCenter(), previousZone, true);
            return RED_MAPPING.targets().get(zone);
        }

        if (x < FIELD_CENTER_X_METERS) {
            return y < FIELD_CENTER_Y_METERS ? BLUE_MIDDLE_TOP_TARGET : BLUE_MIDDLE_BOTTOM_TARGET;
        }
        return y < FIELD_CENTER_Y_METERS ? RED_MIDDLE_TOP_TARGET : RED_MIDDLE_BOTTOM_TARGET;
    }

    private static GridZone selectOuterZone(
            Pose2d robotPose, Pose2d gridCenter, Optional<GridZone> previousZone, boolean isRedSide) {
        double dy = robotPose.getY() - gridCenter.getY();
        double wallDepth = isRedSide ? robotPose.getX() - gridCenter.getX() : gridCenter.getX() - robotPose.getX();

        if (wallDepth <= 0.0) {
            return previousZone.filter(TurretGridSelector::isOuterZone).orElse(defaultOuterCenter(isRedSide));
        }

        if (dy > wallDepth + ZONE_HYSTERESIS_METERS) {
            return isRedSide ? GridZone.RED_OUTER_BOTTOM : GridZone.BLUE_OUTER_BOTTOM;
        }
        if (dy < -wallDepth - ZONE_HYSTERESIS_METERS) {
            return isRedSide ? GridZone.RED_OUTER_TOP : GridZone.BLUE_OUTER_TOP;
        }
        return previousZone
                .filter(TurretGridSelector::isOuterZone)
                .filter(zone -> Math.abs(Math.abs(dy) - wallDepth) <= ZONE_HYSTERESIS_METERS)
                .orElse(defaultOuterCenter(isRedSide));
    }

    private static GridZone defaultOuterCenter(boolean isRedSide) {
        return isRedSide ? GridZone.RED_OUTER_CENTER : GridZone.BLUE_OUTER_CENTER;
    }

    private static boolean isOuterZone(GridZone zone) {
        return switch (zone) {
            case RED_OUTER_TOP,
                    RED_OUTER_CENTER,
                    RED_OUTER_BOTTOM,
                    BLUE_OUTER_TOP,
                    BLUE_OUTER_CENTER,
                    BLUE_OUTER_BOTTOM -> true;
            default -> false;
        };
    }

    private static GridTarget createTagTarget(GridZone zone, int tagId, int[] trimTagIds) {
        Translation2d aimPoint = FIELD_LAYOUT
                .getTagPose(tagId)
                .orElseThrow(() -> new IllegalStateException("Missing AprilTag pose for ID " + tagId))
                .toPose2d()
                .getTranslation();
        return new GridTarget(zone, TargetType.TAG, aimPoint, tagId, trimTagIds);
    }

    private static GridTarget createPointTarget(GridZone zone, Translation2d aimPoint) {
        return new GridTarget(zone, TargetType.POINT, aimPoint, -1, new int[0]);
    }

    public enum GridZone {
        RED_OUTER_TOP,
        RED_OUTER_CENTER,
        RED_OUTER_BOTTOM,
        RED_MIDDLE_TOP,
        RED_MIDDLE_BOTTOM,
        BLUE_MIDDLE_TOP,
        BLUE_MIDDLE_BOTTOM,
        BLUE_OUTER_TOP,
        BLUE_OUTER_CENTER,
        BLUE_OUTER_BOTTOM
    }

    public enum TargetType {
        TAG,
        POINT
    }

    public record GridTarget(
            GridZone zone, TargetType targetType, Translation2d fieldAimPoint, int primaryTagId, int[] trimTagIds) {}

    private record OuterWedgeMapping(Pose2d gridCenter, Map<GridZone, GridTarget> targets) {}
}

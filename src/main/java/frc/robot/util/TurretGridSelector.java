package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Map;
import java.util.Optional;

public final class TurretGridSelector {
    private static final double ZONE_HYSTERESIS_METERS = 0.10;

    private static final AprilTagFieldLayout FIELD_LAYOUT = VisionConstants.aprilTagLayout;

    private static final ReefMapping BLUE_MAPPING = new ReefMapping(
            new Pose2d(4.5366686, 4.0346376, Rotation2d.kZero),
            Map.of(
                    GridZone.LOW_Y,
                    new GridTargetDefinition(27, new int[] {18, 27}),
                    GridZone.CENTER,
                    new GridTargetDefinition(26, new int[] {25, 26}),
                    GridZone.HIGH_Y,
                    new GridTargetDefinition(24, new int[] {21, 24})));

    private static final ReefMapping RED_MAPPING = new ReefMapping(
            new Pose2d(12.0043702, 4.0346376, Rotation2d.kZero),
            Map.of(
                    GridZone.LOW_Y, new GridTargetDefinition(8, new int[] {5, 8}),
                    GridZone.CENTER, new GridTargetDefinition(10, new int[] {10, 10}),
                    GridZone.HIGH_Y, new GridTargetDefinition(11, new int[] {2, 11})));

    private TurretGridSelector() {}

    public static GridTarget select(Pose2d robotPose, Alliance alliance, Optional<GridZone> previousZone) {
        ReefMapping mapping = alliance == Alliance.Red ? RED_MAPPING : BLUE_MAPPING;
        GridZone zone = selectZone(robotPose, alliance, mapping.reefCenter(), previousZone);
        GridTargetDefinition definition = mapping.targets().get(zone);
        Pose3d targetPose = FIELD_LAYOUT
                .getTagPose(definition.primaryTagId())
                .orElseThrow(
                        () -> new IllegalStateException("Missing AprilTag pose for ID " + definition.primaryTagId()));
        return new GridTarget(alliance, zone, definition.primaryTagId(), definition.trimTagIds(), targetPose);
    }

    private static GridZone selectZone(
            Pose2d robotPose, Alliance alliance, Pose2d reefCenter, Optional<GridZone> previousZone) {
        double dy = robotPose.getY() - reefCenter.getY();
        double wallDepth =
                alliance == Alliance.Red ? robotPose.getX() - reefCenter.getX() : reefCenter.getX() - robotPose.getX();

        if (wallDepth <= 0.0) {
            return previousZone.orElse(GridZone.CENTER);
        }

        if (dy > wallDepth + ZONE_HYSTERESIS_METERS) {
            return GridZone.HIGH_Y;
        }
        if (dy < -wallDepth - ZONE_HYSTERESIS_METERS) {
            return GridZone.LOW_Y;
        }
        return previousZone
                .filter(zone -> Math.abs(Math.abs(dy) - wallDepth) <= ZONE_HYSTERESIS_METERS)
                .orElse(GridZone.CENTER);
    }

    public enum GridZone {
        LOW_Y,
        CENTER,
        HIGH_Y
    }

    public record GridTarget(Alliance alliance, GridZone zone, int primaryTagId, int[] trimTagIds, Pose3d targetPose) {}

    private record GridTargetDefinition(int primaryTagId, int[] trimTagIds) {}

    private record ReefMapping(Pose2d reefCenter, Map<GridZone, GridTargetDefinition> targets) {}
}

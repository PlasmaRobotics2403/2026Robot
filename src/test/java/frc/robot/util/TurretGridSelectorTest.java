package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import frc.robot.util.TurretGridSelector.TargetType;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class TurretGridSelectorTest {
    @Test
    void blueOuterWedgeTargetsMatchUpdatedDrawing() {
        assertTarget(
                new Pose2d(3.8, 2.9, Rotation2d.kZero), Optional.empty(), GridZone.BLUE_OUTER_TOP, TargetType.TAG, 18);
        assertTarget(
                new Pose2d(3.8, 4.0, Rotation2d.kZero),
                Optional.empty(),
                GridZone.BLUE_OUTER_CENTER,
                TargetType.TAG,
                26);
        assertTarget(
                new Pose2d(3.8, 5.2, Rotation2d.kZero),
                Optional.empty(),
                GridZone.BLUE_OUTER_BOTTOM,
                TargetType.TAG,
                22);
    }

    @Test
    void redOuterWedgeTargetsMatchUpdatedDrawing() {
        assertTarget(
                new Pose2d(12.8, 2.9, Rotation2d.kZero), Optional.empty(), GridZone.RED_OUTER_TOP, TargetType.TAG, 5);
        assertTarget(
                new Pose2d(12.8, 4.0, Rotation2d.kZero),
                Optional.empty(),
                GridZone.RED_OUTER_CENTER,
                TargetType.TAG,
                10);
        assertTarget(
                new Pose2d(12.8, 5.2, Rotation2d.kZero),
                Optional.empty(),
                GridZone.RED_OUTER_BOTTOM,
                TargetType.TAG,
                2);
    }

    @Test
    void middleBandUsesPointTargets() {
        GridTarget blueTop = TurretGridSelector.select(new Pose2d(5.2, 2.0, Rotation2d.kZero), Optional.empty());
        GridTarget blueBottom = TurretGridSelector.select(new Pose2d(5.2, 6.0, Rotation2d.kZero), Optional.empty());
        GridTarget redTop = TurretGridSelector.select(new Pose2d(11.0, 2.0, Rotation2d.kZero), Optional.empty());
        GridTarget redBottom = TurretGridSelector.select(new Pose2d(11.0, 6.0, Rotation2d.kZero), Optional.empty());

        assertPointTarget(blueTop, GridZone.BLUE_MIDDLE_TOP, 1.0, 1.0);
        assertPointTarget(blueBottom, GridZone.BLUE_MIDDLE_BOTTOM, 1.0, 7.069);
        assertPointTarget(redTop, GridZone.RED_MIDDLE_TOP, 15.541, 1.0);
        assertPointTarget(redBottom, GridZone.RED_MIDDLE_BOTTOM, 15.541, 7.069);
    }

    @Test
    void outerZoneHysteresisPreservesPreviousZoneNearBoundary() {
        GridTarget high = TurretGridSelector.select(
                new Pose2d(3.9, 4.85, Rotation2d.kZero), Optional.of(GridZone.BLUE_OUTER_BOTTOM));
        GridTarget low = TurretGridSelector.select(
                new Pose2d(12.65, 3.25, Rotation2d.kZero), Optional.of(GridZone.RED_OUTER_TOP));

        assertEquals(GridZone.BLUE_OUTER_BOTTOM, high.zone());
        assertEquals(22, high.primaryTagId());
        assertEquals(GridZone.RED_OUTER_TOP, low.zone());
        assertEquals(5, low.primaryTagId());
    }

    @Test
    void outsideOuterDepthKeepsPreviousOuterZone() {
        GridTarget blue = TurretGridSelector.select(
                new Pose2d(5.0, 5.4, Rotation2d.kZero), Optional.of(GridZone.BLUE_OUTER_BOTTOM));
        GridTarget red =
                TurretGridSelector.select(new Pose2d(11.5, 2.8, Rotation2d.kZero), Optional.of(GridZone.RED_OUTER_TOP));

        assertEquals(GridZone.BLUE_MIDDLE_BOTTOM, blue.zone());
        assertEquals(TargetType.POINT, blue.targetType());
        assertEquals(GridZone.RED_MIDDLE_TOP, red.zone());
        assertEquals(TargetType.POINT, red.targetType());
    }

    private static void assertTarget(
            Pose2d pose,
            Optional<GridZone> previousZone,
            GridZone expectedZone,
            TargetType expectedType,
            int expectedTagId) {
        GridTarget target = TurretGridSelector.select(pose, previousZone);
        assertEquals(expectedZone, target.zone());
        assertEquals(expectedType, target.targetType());
        assertEquals(expectedTagId, target.primaryTagId());
    }

    private static void assertPointTarget(
            GridTarget target, GridZone expectedZone, double expectedX, double expectedY) {
        assertEquals(expectedZone, target.zone());
        assertEquals(TargetType.POINT, target.targetType());
        assertEquals(-1, target.primaryTagId());
        assertArrayEquals(new int[0], target.trimTagIds());
        assertEquals(expectedX, target.fieldAimPoint().getX(), 1.0e-9);
        assertEquals(expectedY, target.fieldAimPoint().getY(), 1.0e-9);
    }
}

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class TurretGridSelectorTest {
    @Test
    void blueReefZonesMapToExpectedTags() {
        assertTarget(new Pose2d(3.8, 2.9, Rotation2d.kZero), Alliance.Blue, Optional.empty(), GridZone.LOW_Y, 18);
        assertTarget(new Pose2d(3.8, 4.0, Rotation2d.kZero), Alliance.Blue, Optional.empty(), GridZone.CENTER, 26);
        assertTarget(new Pose2d(3.8, 5.2, Rotation2d.kZero), Alliance.Blue, Optional.empty(), GridZone.HIGH_Y, 22);
    }

    @Test
    void redReefZonesMapToExpectedTags() {
        assertTarget(new Pose2d(12.8, 2.9, Rotation2d.kZero), Alliance.Red, Optional.empty(), GridZone.LOW_Y, 5);
        assertTarget(new Pose2d(12.8, 4.0, Rotation2d.kZero), Alliance.Red, Optional.empty(), GridZone.CENTER, 10);
        assertTarget(new Pose2d(12.8, 5.2, Rotation2d.kZero), Alliance.Red, Optional.empty(), GridZone.HIGH_Y, 2);
    }

    @Test
    void hysteresisKeepsPreviousZoneNearBoundary() {
        GridTarget blue = TurretGridSelector.select(
                new Pose2d(3.8, 4.82, Rotation2d.kZero), Alliance.Blue, Optional.of(GridZone.HIGH_Y));
        GridTarget red = TurretGridSelector.select(
                new Pose2d(12.8, 3.25, Rotation2d.kZero), Alliance.Red, Optional.of(GridZone.LOW_Y));

        assertEquals(GridZone.HIGH_Y, blue.zone());
        assertEquals(22, blue.primaryTagId());
        assertEquals(GridZone.LOW_Y, red.zone());
        assertEquals(5, red.primaryTagId());
    }

    @Test
    void leavingReefDepthFallsBackToCenterOrPreviousZone() {
        GridTarget withPrevious = TurretGridSelector.select(
                new Pose2d(5.0, 5.4, Rotation2d.kZero), Alliance.Blue, Optional.of(GridZone.HIGH_Y));
        GridTarget withoutPrevious =
                TurretGridSelector.select(new Pose2d(5.0, 5.4, Rotation2d.kZero), Alliance.Blue, Optional.empty());

        assertEquals(GridZone.HIGH_Y, withPrevious.zone());
        assertEquals(22, withPrevious.primaryTagId());
        assertEquals(GridZone.CENTER, withoutPrevious.zone());
        assertEquals(26, withoutPrevious.primaryTagId());
    }

    private static void assertTarget(
            Pose2d pose, Alliance alliance, Optional<GridZone> previousZone, GridZone expectedZone, int expectedTagId) {
        GridTarget target = TurretGridSelector.select(pose, alliance, previousZone);
        assertEquals(expectedZone, target.zone());
        assertEquals(alliance, target.alliance());
        assertEquals(expectedTagId, target.primaryTagId());
        assertArrayEquals(expectedTrimTagIds(alliance, expectedZone), target.trimTagIds());
        assertEquals(VisionConstants.aprilTagLayout.getTagPose(expectedTagId).orElseThrow(), target.targetPose());
    }

    private static int[] expectedTrimTagIds(Alliance alliance, GridZone zone) {
        return switch (alliance) {
            case Blue -> switch (zone) {
                case LOW_Y -> new int[] {18, 27};
                case CENTER -> new int[] {25, 26};
                case HIGH_Y -> new int[] {21, 24};
            };
            case Red -> switch (zone) {
                case LOW_Y -> new int[] {5, 8};
                case CENTER -> new int[] {9, 10};
                case HIGH_Y -> new int[] {2, 11};
            };
        };
    }
}

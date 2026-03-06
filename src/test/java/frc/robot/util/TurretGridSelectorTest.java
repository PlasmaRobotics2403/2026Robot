package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class TurretGridSelectorTest {
    @Test
    void blueZonesMapToExpectedTags() {
        assertTarget(new Pose2d(3.8, 2.9, Rotation2d.kZero), Alliance.Blue, Optional.empty(), GridZone.LOW_Y, 18);
        assertTarget(new Pose2d(3.8, 4.0, Rotation2d.kZero), Alliance.Blue, Optional.empty(), GridZone.CENTER, 26);
        assertTarget(new Pose2d(3.8, 5.2, Rotation2d.kZero), Alliance.Blue, Optional.empty(), GridZone.HIGH_Y, 21);
    }

    @Test
    void redZonesMapToExpectedTags() {
        assertTarget(new Pose2d(12.8, 2.9, Rotation2d.kZero), Alliance.Red, Optional.empty(), GridZone.LOW_Y, 5);
        assertTarget(new Pose2d(12.8, 4.0, Rotation2d.kZero), Alliance.Red, Optional.empty(), GridZone.CENTER, 10);
        assertTarget(new Pose2d(12.8, 5.2, Rotation2d.kZero), Alliance.Red, Optional.empty(), GridZone.HIGH_Y, 2);
    }

    @Test
    void outsideReefDepthKeepsPreviousZone() {
        GridTarget blue = TurretGridSelector.select(
                new Pose2d(5.0, 5.4, Rotation2d.kZero), Alliance.Blue, Optional.of(GridZone.HIGH_Y));
        GridTarget red = TurretGridSelector.select(
                new Pose2d(11.5, 2.8, Rotation2d.kZero), Alliance.Red, Optional.of(GridZone.LOW_Y));

        assertEquals(GridZone.HIGH_Y, blue.zone());
        assertEquals(21, blue.primaryTagId());
        assertEquals(GridZone.LOW_Y, red.zone());
        assertEquals(5, red.primaryTagId());
    }

    @Test
    void hysteresisPreservesPreviousZoneNearBoundary() {
        GridTarget high = TurretGridSelector.select(
                new Pose2d(3.9, 4.85, Rotation2d.kZero), Alliance.Blue, Optional.of(GridZone.HIGH_Y));
        GridTarget low = TurretGridSelector.select(
                new Pose2d(12.65, 3.25, Rotation2d.kZero), Alliance.Red, Optional.of(GridZone.LOW_Y));

        assertEquals(GridZone.HIGH_Y, high.zone());
        assertEquals(21, high.primaryTagId());
        assertEquals(GridZone.LOW_Y, low.zone());
        assertEquals(5, low.primaryTagId());
    }

    private static void assertTarget(
            Pose2d pose, Alliance alliance, Optional<GridZone> previousZone, GridZone expectedZone, int expectedTagId) {
        GridTarget target = TurretGridSelector.select(pose, alliance, previousZone);
        assertEquals(expectedZone, target.zone());
        assertEquals(expectedTagId, target.primaryTagId());
    }
}

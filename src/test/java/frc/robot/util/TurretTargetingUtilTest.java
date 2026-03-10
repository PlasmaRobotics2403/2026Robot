package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.TurretGridSelector.GridZone;
import frc.robot.util.TurretTargetingUtil.AimMode;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class TurretTargetingUtilTest {
    @Test
    void turretPivotUsesConfiguredOffsetAtZeroRobotHeading() {
        Pose2d robotPose = new Pose2d(5.0, 3.0, Rotation2d.kZero);
        Translation2d pivot = TurretTargetingUtil.getTurretPivotTranslation(robotPose);

        assertEquals(5.0 + Units.inchesToMeters(-6.25), pivot.getX(), 1e-9);
        assertEquals(3.0 + Units.inchesToMeters(-6.25), pivot.getY(), 1e-9);
    }

    @Test
    void odometryModeIsUsedWhenSelectedTagsAreNotVisible() {
        Pose2d robotPose = new Pose2d(3.8, 4.0, Rotation2d.kZero);
        double currentTurretAngleRad = 0.0;

        var solution = TurretTargetingUtil.calculateTagAimSolution(
                robotPose, Alliance.Blue, Optional.of(GridZone.CENTER), Optional.empty(), currentTurretAngleRad, 0.0);

        assertEquals(AimMode.TAG_ODOMETRY_FALLBACK, solution.aimMode());
        assertFalse(solution.cameraLockActive());
        assertEquals(solution.odometryTargetAngleRad(), solution.finalTargetAngleRad(), 1e-9);
    }

    @Test
    void cameraLockModeTakesOverWhenSelectedTagVisible() {
        Pose2d robotPose = new Pose2d(3.8, 4.0, Rotation2d.kZero);
        Rotation2d tx = Rotation2d.fromDegrees(7.5);
        double currentTurretAngleRad = 0.0;

        var solution = TurretTargetingUtil.calculateTagAimSolution(
                robotPose, Alliance.Blue, Optional.of(GridZone.CENTER), Optional.of(tx), currentTurretAngleRad, 0.0);

        assertEquals(AimMode.TAG_CAMERA_LOCK, solution.aimMode());
        assertTrue(solution.cameraLockActive());
        assertEquals(tx.getRadians(), solution.cameraYawRad(), 1e-9);
        assertEquals(solution.cameraTargetAngleRad(), solution.finalTargetAngleRad(), 1e-9);
        assertEquals(currentTurretAngleRad + tx.getRadians(), solution.cameraTargetAngleRad(), 1e-9);
    }

    @Test
    void cameraTargetUsesDirectTurretLocalCorrection() {
        Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(135.0));
        Rotation2d tx = Rotation2d.fromDegrees(-31.0);
        double currentTurretAngleRad = 0.0;

        var solution = TurretTargetingUtil.calculateTagAimSolution(
                robotPose, Alliance.Blue, Optional.of(GridZone.CENTER), Optional.of(tx), currentTurretAngleRad, 0.0);

        assertEquals(Math.toRadians(-31.0), solution.cameraTargetAngleRad(), 1e-9);
        assertEquals(solution.cameraTargetAngleRad(), solution.finalTargetAngleRad(), 1e-9);
    }

    @Test
    void fieldPointModeAlwaysUsesInterpolationPath() {
        Pose2d robotPose = new Pose2d(4.0, 4.0, Rotation2d.kZero);
        Translation2d fieldPoint = new Translation2d(9.0, 2.0);

        var solution = TurretTargetingUtil.calculateFieldPointAimSolution(robotPose, fieldPoint);

        assertEquals(AimMode.FIELD_POINT_INTERPOLATED, solution.aimMode());
        assertFalse(solution.cameraLockActive());
        assertEquals(solution.odometryTargetAngleRad(), solution.finalTargetAngleRad(), 1e-9);
    }
}

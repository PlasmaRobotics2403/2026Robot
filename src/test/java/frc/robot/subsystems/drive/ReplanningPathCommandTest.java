package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class ReplanningPathCommandTest {
    private static final Pose2d TARGET_POSE = Pose2d.kZero;

    @Test
    void acceptsPoseAtPositiveToleranceBoundary() {
        Pose2d currentPose = new Pose2d(Units.inchesToMeters(1.0), 0.0, Rotation2d.fromDegrees(1.0));

        assertTrue(ReplanningPathCommand.isWithinFinalPoseTolerance(currentPose, TARGET_POSE));
    }

    @Test
    void acceptsPoseAtNegativeToleranceBoundary() {
        Pose2d currentPose = new Pose2d(Units.inchesToMeters(-1.0), 0.0, Rotation2d.fromDegrees(-1.0));

        assertTrue(ReplanningPathCommand.isWithinFinalPoseTolerance(currentPose, TARGET_POSE));
    }

    @Test
    void rejectsTranslationOutsideTolerance() {
        Pose2d currentPose = new Pose2d(Units.inchesToMeters(1.001), 0.0, Rotation2d.fromDegrees(1.0));

        assertFalse(ReplanningPathCommand.isWithinFinalPoseTolerance(currentPose, TARGET_POSE));
    }

    @Test
    void rejectsRotationOutsideTolerance() {
        Pose2d currentPose = new Pose2d(Units.inchesToMeters(1.0), 0.0, Rotation2d.fromDegrees(1.001));

        assertFalse(ReplanningPathCommand.isWithinFinalPoseTolerance(currentPose, TARGET_POSE));
    }

    @Test
    void handlesRotationWraparound() {
        Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(179.5));
        Pose2d targetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179.5));

        assertTrue(ReplanningPathCommand.isWithinFinalPoseTolerance(currentPose, targetPose));
    }

    @Test
    void rejectsNonfinitePose() {
        Pose2d currentPose = new Pose2d(Double.NaN, 0.0, Rotation2d.kZero);

        assertFalse(ReplanningPathCommand.isWithinFinalPoseTolerance(currentPose, TARGET_POSE));
    }
}

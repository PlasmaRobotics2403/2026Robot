package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;

public final class TurretAiming {
    private TurretAiming() {}

    public static double calculateTurretAngleRadians(
            Pose2d robotPose, Translation2d targetField, double currentTurretAngleRad) {
        Translation2d turretField = robotPose
                .getTranslation()
                .plus(TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER.rotateBy(robotPose.getRotation()));

        Translation2d turretToTargetField = targetField.minus(turretField);

        Rotation2d targetFieldBearing = turretToTargetField.getAngle();

        Rotation2d targetRobotBearing = targetFieldBearing.minus(robotPose.getRotation());

        Rotation2d ccwErrorFromTurretZero = targetRobotBearing.minus(TurretConstants.TURRET_ZERO_ROBOT_BEARING);

        Rotation2d turretTarget = Rotation2d.fromRadians(-ccwErrorFromTurretZero.getRadians())
                .plus(TurretConstants.TURRET_AIM_CALIBRATION_OFFSET);

        double targetRad = MathUtil.angleModulus(turretTarget.getRadians());
        return unwrapToTurretRange(targetRad, currentTurretAngleRad);
    }

    private static double unwrapToTurretRange(double targetRad, double currentTurretAngleRad) {
        double minRad = Units.degreesToRadians(TurretConstants.MIN_ANGLE_DEG);
        double maxRad = Units.degreesToRadians(TurretConstants.MAX_ANGLE_DEG);

        double bestTarget = targetRad;
        double bestError = Math.abs(targetRad - currentTurretAngleRad);

        for (double candidate = targetRad - 2.0 * Math.PI;
                candidate <= targetRad + 2.0 * Math.PI;
                candidate += 2.0 * Math.PI) {
            if (candidate < minRad || candidate > maxRad) {
                continue;
            }

            double error = Math.abs(candidate - currentTurretAngleRad);
            if (error < bestError) {
                bestTarget = candidate;
                bestError = error;
            }
        }

        return MathUtil.clamp(bestTarget, minRad, maxRad);
    }
}

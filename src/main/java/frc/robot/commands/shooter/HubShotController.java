package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TurretAiming;
import org.littletonrobotics.junction.Logger;

final class HubShotController {
    private HubShotController() {}

    static boolean updateShotSolution(Shooter shooter, TestTurretSubsystem turret, Drive drive) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hubField = alliance == Alliance.Blue
                ? new Translation2d(Constants.blueHubX, Constants.blueHubY)
                : new Translation2d(Constants.redHubX, Constants.redHubY);
        Translation2d compensatedHubField = drive.getVelocityCompensatedTargetTranslation(hubField);
        double turretPositionRad = turret.getPositionRadians();

        if (!isFinite(hubField) || !isFinite(compensatedHubField) || !Double.isFinite(turretPositionRad)) {
            shooter.stopFlywheel();
            turret.clearUnwinding();
            Logger.recordOutput("Turret/ShootOnMove/ShotSolutionValid", false);
            return false;
        }

        double turretTargetRad =
                TurretAiming.calculateTurretAngleRadians(drive.getPose(), compensatedHubField, turretPositionRad);
        if (!Double.isFinite(turretTargetRad)) {
            shooter.stopFlywheel();
            turret.clearUnwinding();
            Logger.recordOutput("Turret/ShootOnMove/ShotSolutionValid", false);
            return false;
        }

        shooter.runShotFromDistanceToHub(hubField);
        turret.setTargetAngleRadians(turretTargetRad);
        turret.updateUnwinding(requiresUnwind(turretPositionRad, turretTargetRad));

        Logger.recordOutput("Turret/ShootOnMove/ShotSolutionValid", true);
        Logger.recordOutput("Turret/ShootOnMove/UnwindActive", turret.isUnwinding());
        Logger.recordOutput("Turret/ShootOnMove/RawHub", hubField);
        Logger.recordOutput("Turret/ShootOnMove/CompensatedHub", compensatedHubField);
        Logger.recordOutput("Turret/ShootOnMove/CompensationMeters", compensatedHubField.getDistance(hubField));
        Logger.recordOutput("Turret/ShootOnMove/VelocityToMetersConstant", drive.velocityToMetersConstant);
        Logger.recordOutput("Turret/ShootOnMove/FieldVelocity", drive.getFieldRelativeVelocity());

        double robotCenterDistanceMeters = drive.getPose().getTranslation().getDistance(hubField);
        double compensatedShooterDistanceMeters = drive.distanceToTargetMeters(hubField);
        Logger.recordOutput("Turret/ShootOnMove/RobotCenterDistanceMeters", robotCenterDistanceMeters);
        Logger.recordOutput("Turret/ShootOnMove/CompensatedShooterDistanceMeters", compensatedShooterDistanceMeters);
        Logger.recordOutput(
                "Turret/ShootOnMove/ShooterDistanceAdjustmentMeters",
                compensatedShooterDistanceMeters - robotCenterDistanceMeters);
        return true;
    }

    private static boolean isFinite(Translation2d translation) {
        return Double.isFinite(translation.getX()) && Double.isFinite(translation.getY());
    }

    static boolean requiresUnwind(double currentAngleRad, double targetAngleRad) {
        return Double.isFinite(currentAngleRad)
                && Double.isFinite(targetAngleRad)
                && Math.abs(targetAngleRad - currentAngleRad) > Math.PI;
    }
}

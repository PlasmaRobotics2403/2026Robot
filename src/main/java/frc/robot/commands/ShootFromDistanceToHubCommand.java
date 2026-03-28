package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFromDistanceToHubCommand extends Command {
    private final Shooter shooter;
    private final TestTurretSubsystem turret;
    private final Drive drive;

    public ShootFromDistanceToHubCommand(Shooter shooter, TestTurretSubsystem turret, Drive drive) {
        this.shooter = shooter;
        this.turret = turret;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double hubX, hubY;
        if (alliance == Alliance.Blue) {
            hubX = Constants.blueHubX;
            hubY = Constants.blueHubY;
        } else {
            hubX = Constants.redHubX;
            hubY = Constants.redHubY;
        }
        Translation2d hubField = new Translation2d(hubX, hubY);

        shooter.runShotFromDistanceToHub(hubField);
        if ((drive.getRotation().getDegrees() > 90 && drive.getRotation().getDegrees() < 180)
                || (drive.getRotation().getDegrees() < -90
                        && drive.getRotation().getDegrees() > -180)) {
            turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(-10));
        } else if (drive.getRotation().getDegrees() > -90) {
            turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(0));
        } else {
            turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(10));
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

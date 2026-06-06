package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFromDistanceToHubCommandAuto extends Command {
    private final Shooter shooter;
    private final IndexerSubsystem indexer;
    private final Drive drive;
    private final TestTurretSubsystem turret;
    private Timer timer;

    private double time;

    public ShootFromDistanceToHubCommandAuto(
            Shooter shooter, IndexerSubsystem indexer, Drive drive, TestTurretSubsystem turret, double time) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.turret = turret;
        this.time = time;

        this.timer = new Timer();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
        timer.start();
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
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(-10));
            } else {
                turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(8));
            }
        } else if (drive.getRotation().getDegrees() > -90) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(10));
            } else {
                turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(-12));
            }
        } else {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(10));
            } else {
                turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(-10));
            }
        }

        // turret.setTargetAngleRadians(drive.calcTurretAngle(hubField));

        if (timer.hasElapsed(0.5)) {
            indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
            indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
        indexer.setShooterIndexerDutyCycle(0);
        indexer.setSpindexerDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

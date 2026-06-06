package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFromDistanceToHubCommand extends Command {
    private final Shooter shooter;
    private final TestTurretSubsystem turret;
    private final Drive drive;
    private IntakeSubsystem intakeSubsystem;
    private Timer timer;

    public ShootFromDistanceToHubCommand(
            Shooter shooter, TestTurretSubsystem turret, IntakeSubsystem intakeSubsystem, Drive drive) {
        this.shooter = shooter;
        this.turret = turret;
        this.drive = drive;
        this.intakeSubsystem = intakeSubsystem;
        this.timer = new Timer();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intakeSubsystem.setPivotTargetDegrees(IntakeConstants.STOW_DEG);
    }

    @Override
    public void execute() {

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (timer.hasElapsed(1.5)) {
            timer.reset();
            timer.start();
        }

        if (timer.hasElapsed(0.5)) {
            intakeSubsystem.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
        }

        if (timer.hasElapsed(1)) {
            intakeSubsystem.setPivotTargetDegrees(IntakeConstants.STOW_DEG);
        }

        if (Math.toDegrees(intakeSubsystem.getPivotPositionRadians()) > 40) {
            intakeSubsystem.setRollerPercent(0.25);
        } else {
            intakeSubsystem.setRollerPercent(0);
        }

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
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
        intakeSubsystem.setPivotTargetDegrees(IntakeConstants.STOW_DEG);
        intakeSubsystem.setRollerPercent(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

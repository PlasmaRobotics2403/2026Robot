package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShuttleShot extends Command {
    private final Shooter shooter;
    private double currentYPos;
    private final TestTurretSubsystem turret;
    private final Drive drive;

    public ShuttleShot(Shooter shooter, DoubleSupplier currentYPos, TestTurretSubsystem turret, Drive drive) {
        this.shooter = shooter;
        this.currentYPos = currentYPos.getAsDouble();
        this.turret = turret;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double targetX, targetY;
        if (alliance == Alliance.Blue) {
            if (currentYPos < 4) {
                targetX = Constants.blueShuttleX;
                targetY = drive.getPose().getY();
            } else {
                targetX = Constants.blueShuttleX;
                targetY = drive.getPose().getY();
            }
        } else {
            if (currentYPos < 4) {
                targetX = Constants.redShuttleX;
                targetY = drive.getPose().getY();
            } else {
                targetX = Constants.redShuttleX;
                targetY = drive.getPose().getY();
            }
        }

        shooter.runShot(1000, 58);
        turret.setTargetAngleRadians(Math.toRadians(drive.getRotation().getDegrees()) + Math.toRadians(-90));
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

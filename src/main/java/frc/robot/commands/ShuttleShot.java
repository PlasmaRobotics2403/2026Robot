package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
        boolean isRobotSideNearBlue = drive.getPose().getX() <= 7.8
                && alliance == Alliance.Blue; // checks to see if the robot is near or far from the alliane station
        boolean isRobotSideNearRed = drive.getPose().getX() >= 9
                && alliance == Alliance.Red; // checks to see if the robot is near or far from the alliane station

        if (isRobotSideNearBlue || isRobotSideNearRed) {
            shootNear();
        } else {
            shootFar();
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

    private void shootNear() {
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
        shooter.runShot(800, 55);
        double turretTargetDeg;
        if (alliance == Alliance.Blue) {
            turretTargetDeg = drive.getRotation().getDegrees() - 80.0;
        } else {
            turretTargetDeg = drive.getRotation().getDegrees() - 270.0;
        }
        turretTargetDeg = MathUtil.inputModulus(turretTargetDeg, -180.0, 180.0);
        turret.setTargetAngleRadians(Math.toRadians(turretTargetDeg) - Math.toRadians(10.0));
    }

    private void shootFar() {
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
        shooter.runShot(1000, 85);
        double turretTargetDeg;
        if (alliance == Alliance.Blue) {
            turretTargetDeg = drive.getRotation().getDegrees() - 80.0;
        } else {
            turretTargetDeg = drive.getRotation().getDegrees() - 270.0;
        }
        turretTargetDeg = MathUtil.inputModulus(turretTargetDeg, -180.0, 180.0);
        turret.setTargetAngleRadians(Math.toRadians(turretTargetDeg) - Math.toRadians(10.0));
    }
}

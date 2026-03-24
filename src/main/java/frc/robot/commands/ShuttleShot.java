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
                targetX = Constants.blueNearShuttleX;
                targetY = Constants.blueNearShuttleY;
            } else {
                targetX = Constants.blueFarShuttleX;
                targetY = Constants.blueFarShuttleY;
            }
        } else {
            if (currentYPos < 4) {
                targetX = Constants.redNearShuttleX;
                targetY = Constants.redNearShuttleY;
            } else {
                targetX = Constants.redFarShuttleX;
                targetY = Constants.redFarShuttleY;
            }
        }

        turret.setTargetAngleRadians(drive.calcTurretAngle(new Translation2d(targetX, targetY)) + Math.toRadians(-10));
        shooter.runShuttleShot(new Translation2d(targetX, targetY));
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

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShuttleShot extends Command {
    private final Shooter shooter;
    private double currentYPos;
    private double nearAngleOffset = 6;
    private double farAngleOffset = 20;
    private double nearSpeed = 35;
    private double farSpeed = 85;
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
        // SmartDashboard.putNumber("Shooter/Shuttle Near Angle Offset", nearAngleOffset);
        // SmartDashboard.putNumber("Shooter/Shuttle Far Angle Offset", farAngleOffset);

        nearAngleOffset = SmartDashboard.getNumber("Shooter/Shuttle Near Angle Offset", nearAngleOffset);
        farAngleOffset = SmartDashboard.getNumber("Shooter/Shuttle Far Angle Offset", farAngleOffset);
        nearSpeed = SmartDashboard.getNumber("Shooter/Shuttle Near Speed", nearSpeed);
        farSpeed = SmartDashboard.getNumber("Shooter/Shuttle Far Speed", farSpeed);
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

        if (alliance == Alliance.Blue) {
            if (currentYPos > 4) {
                nearAngleOffset = nearAngleOffset * -1;
            }
        } else {
            if (currentYPos < 4) {
                nearAngleOffset = nearAngleOffset * -1;
            }
        }
        shooter.runShot(800, nearSpeed);
        double turretTargetDeg;
        if (alliance == Alliance.Blue) {
            turretTargetDeg = drive.getRotation().getDegrees() + nearAngleOffset - 80.0;
        } else {
            turretTargetDeg = drive.getRotation().getDegrees() + nearAngleOffset - 270.0;
        }
        turretTargetDeg = MathUtil.inputModulus(turretTargetDeg, -180.0, 180.0);
        turret.setTargetAngleRadians(Math.toRadians(turretTargetDeg) - Math.toRadians(10.0));
    }

    private void shootFar() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Blue) {
            if (currentYPos > 4) {
                farAngleOffset = farAngleOffset * -1;
            }
        } else {
            if (currentYPos < 4) {
                farAngleOffset = farAngleOffset * -1;
            }
        }
        shooter.runShot(1000, farSpeed);
        double turretTargetDeg;
        if (alliance == Alliance.Blue) {
            turretTargetDeg = drive.getRotation().getDegrees() + farAngleOffset - 90.0;
        } else {
            turretTargetDeg = drive.getRotation().getDegrees() + farAngleOffset - 270.0;
        }
        turretTargetDeg = MathUtil.inputModulus(turretTargetDeg, -180.0, 180.0);
        turret.setTargetAngleRadians(Math.toRadians(turretTargetDeg) - Math.toRadians(10.0));
    }
}

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TurretAiming;
import org.littletonrobotics.junction.Logger;

public class ShootFromDistanceToHubCommand extends Command {
    // Tune this. It should be the horizontal speed of the ball/note leaving the shooter.
    private static final double PROJECTILE_SPEED_METERS_PER_SEC = 10.0;

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

        double hubX;
        double hubY;
        if (alliance == Alliance.Blue) {
            hubX = Constants.blueHubX;
            hubY = Constants.blueHubY;
        } else {
            hubX = Constants.redHubX;
            hubY = Constants.redHubY;
        }

        Translation2d hubField = new Translation2d(hubX, hubY);
        Translation2d compensatedHubField = calculateMovingShotTarget(hubField);

        // Keep shooter distance based on the real hub location.
        shooter.runShotFromDistanceToHub(hubField);

        // double turretTargetRad = TurretAiming.calculateTurretAngleRadians(
        //         drive.getPose(),
        //         compensatedHubField,
        //         turret.getPositionRadians());

        double turretTargetRad =
                TurretAiming.calculateTurretAngleRadians(drive.getPose(), hubField, turret.getPositionRadians());

        turret.setTargetAngleRadians(turretTargetRad);

        Logger.recordOutput("Turret/ShootOnMove/RawHub", hubField);
        Logger.recordOutput("Turret/ShootOnMove/CompensatedHub", compensatedHubField);
        Logger.recordOutput("Turret/ShootOnMove/ProjectileSpeedMetersPerSec", PROJECTILE_SPEED_METERS_PER_SEC);
    }

    private Translation2d calculateMovingShotTarget(Translation2d hubField) {
        Pose2d robotPose = drive.getPose();

        Translation2d turretOffsetField =
                TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER.rotateBy(robotPose.getRotation());

        Translation2d turretField = robotPose.getTranslation().plus(turretOffsetField);

        Translation2d turretToHub = hubField.minus(turretField);

        ChassisSpeeds fieldVelocity = drive.getFieldRelativeVelocity();

        Translation2d robotVelocityField =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        double omegaRadPerSec = fieldVelocity.omegaRadiansPerSecond;

        Translation2d turretVelocityFromRobotRotation = new Translation2d(
                -omegaRadPerSec * turretOffsetField.getY(), omegaRadPerSec * turretOffsetField.getX());

        Translation2d turretVelocityField = robotVelocityField.plus(turretVelocityFromRobotRotation);

        double flightTimeSec =
                solveShotInterceptTime(turretToHub, turretVelocityField, PROJECTILE_SPEED_METERS_PER_SEC);

        Translation2d aimVector = turretToHub.minus(turretVelocityField.times(flightTimeSec));

        Translation2d compensatedHubField = turretField.plus(aimVector);

        Logger.recordOutput("Turret/ShootOnMove/TurretField", turretField);
        Logger.recordOutput("Turret/ShootOnMove/TurretVelocityField", turretVelocityField);
        Logger.recordOutput("Turret/ShootOnMove/FlightTimeSec", flightTimeSec);

        return compensatedHubField;
    }

    private double solveShotInterceptTime(
            Translation2d turretToHub, Translation2d turretVelocityField, double projectileSpeedMetersPerSec) {
        double rx = turretToHub.getX();
        double ry = turretToHub.getY();
        double vx = turretVelocityField.getX();
        double vy = turretVelocityField.getY();
        double s = projectileSpeedMetersPerSec;

        double a = vx * vx + vy * vy - s * s;
        double b = -2.0 * (rx * vx + ry * vy);
        double c = rx * rx + ry * ry;

        if (Math.abs(a) < 1e-9) {
            if (Math.abs(b) < 1e-9) {
                return Math.sqrt(c) / s;
            }

            double linearTime = -c / b;
            return linearTime > 0.0 ? linearTime : Math.sqrt(c) / s;
        }

        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return Math.sqrt(c) / s;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
        double t2 = (-b + sqrtDiscriminant) / (2.0 * a);

        if (t1 > 0.0 && t2 > 0.0) {
            return Math.min(t1, t2);
        } else if (t1 > 0.0) {
            return t1;
        } else if (t2 > 0.0) {
            return t2;
        } else {
            return Math.sqrt(c) / s;
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

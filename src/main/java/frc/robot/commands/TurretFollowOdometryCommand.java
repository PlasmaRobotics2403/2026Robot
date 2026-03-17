package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.TurretGridSelector;
import frc.robot.util.TurretGridSelector.GridTarget;
import java.util.Optional;

public class TurretFollowOdometryCommand extends Command {

    private Vision vision;
    private TestTurretSubsystem turret;
    private Drive drive;

    private int cameraIndex;
    private Timer timer = new Timer();

    private AprilTagFieldLayout layout;

    public TurretFollowOdometryCommand(Vision vision, TestTurretSubsystem turret, Drive drive, int cameraIndex) {
        this.vision = vision;
        this.turret = turret;
        this.drive = drive;
        this.cameraIndex = cameraIndex;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        layout = VisionConstants.aprilTagLayout;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        GridTarget target = TurretGridSelector.select(robotPose, alliance, Optional.empty());

        int tagID = target.primaryTagId();

        turret.setTargetAngleRadians(aimUsingOdometryCenterHub(robotPose, alliance));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

    private double aimUsingOdometryCenterHub(Pose2d robotPose, Alliance alliance) {
        double hubX, hubY;
        if (alliance == Alliance.Blue) {
            hubX = Constants.blueHubX;
            hubY = Constants.blueHubY;
        } else {
            hubX = Constants.redHubX;
            hubY = Constants.redHubY;
        }

        Rotation2d robotHeading = robotPose.getRotation();
        Translation2d robotTranslation = robotPose.getTranslation();

        Translation2d turretOffsetRobot = TurretConstants.TURRET_PIVOT_FROM_ROBOT_CENTER;
        Translation2d turretOffsetField = turretOffsetRobot.rotateBy(robotHeading);
        Translation2d turretField = robotTranslation.plus(turretOffsetField);

        Translation2d hubField = new Translation2d(hubX, hubY);
        Translation2d turretToHubField = hubField.minus(turretField);

        Rotation2d targetFieldAngle = turretToHubField.getAngle();
        Rotation2d targetRobotAngle = targetFieldAngle.minus(robotHeading);

        Rotation2d turretZeroAngle = Rotation2d.fromDegrees(90);

        Rotation2d turretCommand = targetRobotAngle.minus(turretZeroAngle);

        double targetRadians = MathUtil.angleModulus(turretCommand.getRadians());
        targetRadians = applyUnwind(targetRadians);

        return targetRadians;
    }

    private double applyUnwind(double targetAngle) {
        double min = Math.toRadians(TurretConstants.MIN_ANGLE_DEG);
        double max = Math.toRadians(TurretConstants.MAX_ANGLE_DEG);

        while (targetAngle < min - Math.toRadians(3)) {
            targetAngle += 2 * Math.PI;
        }

        while (targetAngle > max + Math.toRadians(3)) {
            targetAngle -= 2 * Math.PI;
        }

        return targetAngle;
    }
}

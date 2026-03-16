package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.TurretGridSelector;
import frc.robot.util.TurretGridSelector.GridTarget;
import java.util.Optional;

public class TurretFollowCommand extends Command {

    private Vision vision;
    private TestTurretSubsystem turret;
    private Drive drive;

    private int cameraIndex;
    private Timer timer = new Timer();
    double offset = 0;

    public TurretFollowCommand(Vision vision, TestTurretSubsystem turret, Drive drive, int cameraIndex) {
        this.vision = vision;
        this.turret = turret;
        this.drive = drive;
        this.cameraIndex = cameraIndex;

        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        SmartDashboard.putNumber("Turret/Follow/Offset", offset);
        offset = SmartDashboard.getNumber("Turret/Follow/Offset", 0);
        Pose2d robotPose = drive.getPose();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        GridTarget target = TurretGridSelector.select(robotPose, alliance, Optional.empty());

        int tagID = target.primaryTagId();
        Rotation2d tx = vision.getTargetXForTag(cameraIndex, tagID);
        double angle = applyUnwind(turret.getPositionRadians() + tx.getRadians());

        angle += offset;
        angle = applyUnwind(angle);

        boolean seesTag = vision.seesTag(cameraIndex, tagID);
        SmartDashboard.putNumber("Turret PID/ObometryPose", aimUsingOdometry(robotPose, tagID));
        SmartDashboard.putNumber("Turret PID/ObometryPose", angle);
        if (seesTag) {
            turret.setTargetAngleRadians(angle);
        } else {
            // turret.setTargetAngleRadians(aimUsingOdometry(robotPose, tagID));
        }
        SmartDashboard.putNumber("Turret/Follow/TxDeg", tx.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private double aimUsingOdometry(Pose2d robotPose, int tagID) {
        AprilTagFieldLayout layout = VisionConstants.aprilTagLayout;
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(tagID);

        if (tagPoseOptional.isEmpty()) {
            return turret.getPositionRadians();
        }

        Pose2d tagPose = tagPoseOptional.get().toPose2d();
        Translation2d robot = robotPose.getTranslation();
        Translation2d tag = tagPose.getTranslation();

        double fieldAngle = Math.atan2(tag.getY() - robot.getY(), tag.getX() - robot.getX());

        double turretAngle =
                fieldAngle - robotPose.getRotation().getRadians() - Math.toRadians(90); // turret faces left
        turretAngle = MathUtil.angleModulus(turretAngle);
        turretAngle = applyUnwind(turretAngle);

        return turretAngle;
    }

    private double applyUnwind(double targetAngle) {
        double min = Math.toRadians(TurretConstants.MIN_ANGLE_DEG);
        double max = Math.toRadians(TurretConstants.MAX_ANGLE_DEG);

        while (targetAngle < min) {
            targetAngle += 2 * Math.PI;
        }

        while (targetAngle > max) {
            targetAngle -= 2 * Math.PI;
        }

        return targetAngle;
    }
}

package frc.robot.commands.turret;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;

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

        turret.setTargetAngleRadians(drive.calcTurretAngle(hubField) + Math.toRadians(-10));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}

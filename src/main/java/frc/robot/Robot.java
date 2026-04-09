package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.currentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();
        robotContainer.updateDashboardField();

        Threads.setCurrentThreadPriority(false, 10);

        SmartDashboard.putNumber("GridSelector/gridTargetID", robotContainer.getGridSelectorTagID());
        SmartDashboard.putString("GridSelector/targetGrid", robotContainer.getGridZone());
        SmartDashboard.putNumber("Turret/TagDistance", robotContainer.calculateDistanceToHubMeters());

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            SmartDashboard.putString("GameColor", "Blue");
        } else {
            SmartDashboard.putString("GameColor", "Red");
        }

        SmartDashboard.putNumber("RobotY", robotContainer.getRobotY());
    }

    @Override
    public void disabledInit() {
        robotContainer.resetSimulation();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.stopShooter();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        robotContainer.updateSimulation();
        Pose2d simPose = robotContainer.getSimulationPose();
        Pose2d odometryPose = robotContainer.getDrivePose();
        Logger.recordOutput("FieldSimulation/OdometryPose", odometryPose);
        Logger.recordOutput(
                "FieldSimulation/PoseErrorMeters", simPose.getTranslation().getDistance(odometryPose.getTranslation()));
        Logger.recordOutput(
                "FieldSimulation/PoseErrorDeg",
                simPose.getRotation().minus(odometryPose.getRotation()).getDegrees());
    }
}

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDs;
import java.util.Optional;
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
        // robotContainer.ledsOn();
    }

    @Override
    public void robotPeriodic() {
        robotContainer.getLEDs().periodic();
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
    public void disabledPeriodic() {
        // robotContainer.getLEDs().rainbow();
        // robotContainer.getLEDs().setHSV(0, 255, 128);4
        // robotContainer.getLEDs().rainbow();
        robotContainer.getLEDs().setState(LEDs.LEDState.BOGO);
    }

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
    public void teleopPeriodic() {
        if (isHubActive()) {
            robotContainer.getLEDs().setState(LEDs.LEDState.NOPEICE);
        } else {
            robotContainer.getLEDs().setState(LEDs.LEDState.SEETARGET);
        }
    }

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

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}

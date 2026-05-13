package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
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
    private static final double HUB_WARNING_SECONDS = 3.0;
    private static final double HUB_BLINK_INTERVAL_SECONDS = 0.5;

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
        robotContainer.getLEDs().setState(LEDs.LEDState.NOPEICE);

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
        if (isHubWarningWindow()) {
            boolean purpleOn = ((int) (Timer.getFPGATimestamp() / HUB_BLINK_INTERVAL_SECONDS)) % 2 == 0;
            if (purpleOn) {
                robotContainer.getLEDs().setState(LEDs.LEDState.NOPEICE);
            } else {
                robotContainer.getLEDs().setState(LEDs.LEDState.OFF);
            }
        } else if (isHubActive()) {
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

    private boolean isHubActiveAtTime(double matchTime, boolean shift1Active) {
        if (matchTime > 130) {
            return true;
        } else if (matchTime > 105) {
            return shift1Active;
        } else if (matchTime > 80) {
            return !shift1Active;
        } else if (matchTime > 55) {
            return shift1Active;
        } else if (matchTime > 30) {
            return !shift1Active;
        } else {
            return true;
        }
    }

    private double getTimeUntilHubActiveSeconds() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty() || !DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled()) {
            return -1.0;
        }

        boolean redInactiveFirst;
        if (!DriverStation.isFMSAttached()) {
            // Practice mode behavior requested
            redInactiveFirst = true;
        } else {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.isEmpty()) {
                return -1.0;
            }
            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> {
                    return -1.0;
                }
            }
        }

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            return -1.0;
        }

        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (isHubActiveAtTime(matchTime, shift1Active)) {
            return -1.0;
        }

        if (matchTime > 105) {
            return matchTime - 105;
        } else if (matchTime > 80) {
            return matchTime - 80;
        } else if (matchTime > 55) {
            return matchTime - 55;
        } else if (matchTime > 30) {
            return matchTime - 30;
        }

        return -1.0;
    }

    private boolean isHubWarningWindow() {
        double timeUntilActive = getTimeUntilHubActiveSeconds();
        return timeUntilActive >= 0.0 && timeUntilActive <= HUB_WARNING_SECONDS;
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        boolean redInactiveFirst;
        if (!DriverStation.isFMSAttached()) {
            redInactiveFirst = true;
        } else {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.isEmpty()) {
                return false;
            }
            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> {
                    return false;
                }
            }
        }

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            return false;
        }

        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        return isHubActiveAtTime(matchTime, shift1Active);
    }
}

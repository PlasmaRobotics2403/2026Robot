package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Locale;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    public enum ControlMode {
        IDLE,
        FLYWHEEL_VELOCITY,
        FLYWHEEL_DUTY,
        HOOD_DUTY,
        HOOD_POSITION
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double tuningFarShoot = 1;
    double tuningFarShootDistance = 5;

    double accumulatedTime = 0.0;
    private double rpsOffset = 1;
    private ControlMode controlMode = ControlMode.IDLE;
    private double flywheelSetpointRps = 0.0;
    private double hoodSetpointRotations = 0.0;
    private double flywheelPidP = ShooterConstants.FLYWHEEL_KP;
    private double flywheelPidI = ShooterConstants.FLYWHEEL_KI;
    private double flywheelPidD = ShooterConstants.FLYWHEEL_KD;
    private double flywheelPidV = ShooterConstants.FLYWHEEL_KV;
    private double hoodPidP = ShooterConstants.HOOD_KP;
    private double hoodPidI = ShooterConstants.HOOD_KI;
    private double hoodPidD = ShooterConstants.HOOD_KD;
    private double hoodDutySetpoint = 0.0;
    private final Debouncer flywheelReadyDebouncer =
            new Debouncer(ShooterConstants.FLYWHEEL_READY_DEBOUNCE_SECONDS, DebounceType.kRising);
    private double hoodStallAccumulatedTimeSec = 0.0;
    private boolean hoodStallDetected = false;
    private boolean hoodStallLatched = false;
    private boolean hoodResetPendingAfterStall = false;
    private double lastPeriodicTimestampSec = 0.0;
    private boolean dashboardTargetsResetAfterBoot = false;
    private final DoubleSupplier tuningDistanceSupplier;

    private Drive drive;

    // public Shooter(ShooterIO io, Drive drive) {
    //     this(
    //             io,
    //             () -> SmartDashboard.getNumber(
    //                     ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_KEY,
    //                     ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT),
    //             () -> SmartDashboard.getNumber(
    //                     ShooterConstants.TUNING_HUB_DISTANCE_METERS_DASHBOARD_KEY,
    //                     ShooterConstants.TUNING_HUB_DISTANCE_METERS_DASHBOARD_DEFAULT),
    //             true);
    //     this.drive = drive;
    // }

    public Shooter(ShooterIO io, DoubleSupplier tuningDistanceSupplier, Drive drive) {
        this(io, tuningDistanceSupplier, true);
        this.drive = drive;
        SmartDashboard.putNumber("Shooter/rpsOffset", rpsOffset);
    }

    // Shooter(ShooterIO io, boolean initializeDashboard) {
    //     this(
    //             io,
    //             () -> SmartDashboard.getNumber(
    //                     ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_KEY,
    //                     ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT),
    //             () -> SmartDashboard.getNumber(
    //                     ShooterConstants.TUNING_HUB_DISTANCE_METERS_DASHBOARD_KEY,
    //                     ShooterConstants.TUNING_HUB_DISTANCE_METERS_DASHBOARD_DEFAULT),
    //             initializeDashboard);
    // }

    Shooter(ShooterIO io, DoubleSupplier tuningDistanceSupplier, boolean initializeDashboard) {
        this.io = io;
        this.tuningDistanceSupplier = tuningDistanceSupplier;
        this.hoodSetpointRotations = hoodDegreesToMotorRotations(ShooterConstants.HOOD_ZERO_ANGLE_DEGREES);

        io.setHoodPositionRotations(hoodSetpointRotations);

        if (!initializeDashboard) {
            return;
        }

        SmartDashboard.putNumber(ShooterConstants.TUNING_FAR_SHOT, tuningFarShoot);
        SmartDashboard.putNumber(ShooterConstants.TUNING_FAR_SHOT_DISTANCE, tuningFarShootDistance);

        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kP", flywheelPidP);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kI", flywheelPidI);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kD", flywheelPidD);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kV", flywheelPidV);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kP", hoodPidP);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kI", hoodPidI);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kD", hoodPidD);
        SmartDashboard.putNumber("Shooter/Shuttle Near Angle Offset", 10);
        SmartDashboard.putNumber("Shooter/Shuttle Far Angle Offset", 6);
        SmartDashboard.putNumber("Shooter/Shuttle Near Speed", 50);
        SmartDashboard.putNumber("Shooter/Shuttle Far Speed", 85);

        SmartDashboard.putNumber("Shooter/rpsOffset", rpsOffset);
        SmartDashboard.putNumber(
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.FLYWHEEL_DUTY_DASHBOARD_KEY, ShooterConstants.FLYWHEEL_DUTY_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_KEY,
                ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_TAG_DISTANCE_METERS_DASHBOARD_KEY,
                ShooterConstants.TUNING_TAG_DISTANCE_METERS_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_KEY,
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        SmartDashboard.putString(
                ShooterConstants.TUNING_SAMPLE_ROW_DASHBOARD_KEY,
                formatSampleRow(
                        ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT,
                        ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_DEFAULT,
                        ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT));
    }

    @Override
    public void periodic() {
        double nowSec = Timer.getFPGATimestamp();
        double dtSec = lastPeriodicTimestampSec > 0.0 ? nowSec - lastPeriodicTimestampSec : 0.02;
        lastPeriodicTimestampSec = nowSec;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

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

        SmartDashboard.putNumber("Shooter/distanceToTarget", drive.distanceToTargetMeters(hubField));

        SmartDashboard.putNumber("Shooter/rpsOffset", rpsOffset);
        forceDashboardTargetsResetAfterBoot();
        io.updateInputs(inputs);
        updateHoodStallDetection(dtSec);
        Logger.processInputs("Shooter", inputs);
        updateDashboardTuning();

        Logger.recordOutput("Shooter/ControlMode", controlMode.toString());
        Logger.recordOutput("Shooter/FlywheelSetpointRps", flywheelSetpointRps);
        Logger.recordOutput("Shooter/Flywheel/TargetRps", flywheelSetpointRps);
        Logger.recordOutput("Shooter/Flywheel/CurrentRps", inputs.flywheelVelocityRps);

        Logger.recordOutput(
                "Shooter/FlywheelAtSpeed",
                atFlywheelSpeed(flywheelSetpointRps, ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS));
        Logger.recordOutput(
                "Shooter/Flywheel/ReadyLowerBoundRps",
                flywheelSetpointRps - ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS);
        Logger.recordOutput(
                "Shooter/Flywheel/ReadyUpperBoundRps",
                flywheelSetpointRps + ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS);
        Logger.recordOutput("Shooter/HoodSetpointRotations", hoodSetpointRotations);
        Logger.recordOutput(
                "Shooter/HoodAtTarget",
                atHoodPositionRotations(hoodSetpointRotations, ShooterConstants.HOOD_POSITION_TOLERANCE_ROTATIONS));

        Logger.recordOutput("Shooter/Flywheel/PID/kP", flywheelPidP);
        Logger.recordOutput("Shooter/Flywheel/PID/kI", flywheelPidI);
        Logger.recordOutput("Shooter/Flywheel/PID/kD", flywheelPidD);
        Logger.recordOutput("Shooter/Flywheel/PID/kV", flywheelPidV);
        Logger.recordOutput("Shooter/Flywheel/LeaderVelocityRps", inputs.flywheelLeaderVelocityRps);
        Logger.recordOutput("Shooter/Flywheel/FollowerVelocityRps", inputs.flywheelFollowerVelocityRps);
        Logger.recordOutput("Shooter/Hood/PID/kP", hoodPidP);
        Logger.recordOutput("Shooter/Hood/PID/kI", hoodPidI);
        Logger.recordOutput("Shooter/Hood/PID/kD", hoodPidD);
        Logger.recordOutput("Shooter/Hood/CurrentAngleDeg", getHoodAngleDegrees());
        Logger.recordOutput("Shooter/Hood/TargetRotationsFromIO", inputs.hoodTargetPositionRotations);
        Logger.recordOutput("Shooter/Hood/CurrentAmps", inputs.hoodCurrentAmps);
        Logger.recordOutput("Shooter/Hood/AppliedVolts", inputs.hoodAppliedVolts);
        Logger.recordOutput("Shooter/Hood/AbsCurrentAmps", Math.abs(inputs.hoodCurrentAmps));
        Logger.recordOutput("Shooter/Hood/AbsAppliedVolts", Math.abs(inputs.hoodAppliedVolts));
        Logger.recordOutput("Shooter/Hood/StallDetected", hoodStallDetected);
        Logger.recordOutput("Shooter/Hood/StallLatched", hoodStallLatched);
        Logger.recordOutput("Shooter/Hood/StallAccumulatedTimeSec", hoodStallAccumulatedTimeSec);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "Active kP", flywheelPidP);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "Active kI", flywheelPidI);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "Active kD", flywheelPidD);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "Active kV", flywheelPidV);

        rpsOffset = SmartDashboard.getNumber("Shooter/rpsOffset", rpsOffset);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "Active kP", hoodPidP);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "Active kI", hoodPidI);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "Active kD", hoodPidD);
        SmartDashboard.putNumber("Shooter/Flywheel/CurrentRps", inputs.flywheelVelocityRps);
        SmartDashboard.putNumber("Shooter/Flywheel/ActiveTargetRps", flywheelSetpointRps);
        SmartDashboard.putNumber("Shooter/Hood/CurrentRotations", inputs.hoodPositionRotations);
        SmartDashboard.putNumber("Shooter/Hood/TargetRotations", inputs.hoodTargetPositionRotations);
        SmartDashboard.putNumber("Shooter/Hood/CurrentDeg", getHoodAngleDegrees());
        SmartDashboard.putNumber("Shooter/Hood/CurrentAmps", inputs.hoodCurrentAmps);
        SmartDashboard.putNumber("Shooter/Hood/AppliedVolts", inputs.hoodAppliedVolts);
        SmartDashboard.putNumber("Shooter/Hood/RPS", inputs.hoodVelocityRps);
        SmartDashboard.putNumber("Shooter/Hood/AbsCurrentAmps", Math.abs(inputs.hoodCurrentAmps));
        SmartDashboard.putNumber("Shooter/Hood/AbsAppliedVolts", Math.abs(inputs.hoodAppliedVolts));
        SmartDashboard.putBoolean("Shooter/Hood/StallDetected", hoodStallDetected);
        SmartDashboard.putBoolean("Shooter/Hood/StallLatched", hoodStallLatched);

        // SmartDashboard.putNumber("Turret/TagDistance", tuningDistanceSupplier.getAsDouble());
        // SmartDashboard.putNumber("", 0);
        updateTuningDashboard();
    }

    private void updateHoodStallDetection(double dtSec) {
        boolean hoodControlActive = controlMode == ControlMode.HOOD_DUTY || controlMode == ControlMode.HOOD_POSITION;
        boolean hasMeaningfulCommand =
                Math.abs(inputs.hoodAppliedVolts) >= ShooterConstants.HOOD_STALL_MIN_APPLIED_VOLTS;
        double hoodPositionErrorRotations = Math.abs(inputs.hoodTargetPositionRotations - inputs.hoodPositionRotations);
        double hoodClosedLoopErrorRotations = Math.abs(inputs.hoodClosedLoopErrorRotations);

        if (controlMode == ControlMode.HOOD_POSITION) {
            hasMeaningfulCommand = hasMeaningfulCommand
                    && (hoodPositionErrorRotations >= ShooterConstants.HOOD_STALL_POSITION_ERROR_THRESHOLD_ROTATIONS
                            || hoodClosedLoopErrorRotations
                                    >= ShooterConstants.HOOD_STALL_POSITION_ERROR_THRESHOLD_ROTATIONS
                            || Math.abs(inputs.hoodAppliedVolts) >= ShooterConstants.HOOD_STALL_HARD_PUSH_VOLTS);
        }

        boolean lowVelocity = Math.abs(inputs.hoodVelocityRps) <= ShooterConstants.HOOD_STALL_VELOCITY_THRESHOLD_RPS;
        boolean highCurrent = Math.abs(inputs.hoodCurrentAmps) >= ShooterConstants.HOOD_STALL_CURRENT_THRESHOLD_AMPS;
        boolean lowCurrent = Math.abs(inputs.hoodCurrentAmps) >= 0.9;
        boolean belowPos =
                inputs.hoodPositionRotations < ShooterConstants.HOOD_STALL_POSITION_ERROR_THRESHOLD_ROTATIONS;
        boolean stallCondition = (lowVelocity && highCurrent) || (lowVelocity && lowCurrent && belowPos);

        if (stallCondition) {
            hoodStallAccumulatedTimeSec += Math.max(0.0, dtSec);
            if (hoodStallAccumulatedTimeSec >= ShooterConstants.HOOD_STALL_DETECTION_TIME_SEC) {
                hoodStallDetected = true;
                if (!hoodStallLatched) {
                    hoodStallLatched = true;
                    hoodResetPendingAfterStall = true;
                    DriverStation.reportWarning(
                            "Shooter hood stall detected. Hood motor stopped for protection.", false);
                }
            }
        } else {
            hoodStallAccumulatedTimeSec = 0.0;
            hoodStallDetected = false;
        }

        hoodStallDetected = stallCondition;
        if (stallCondition) {
            accumulatedTime += dtSec;
        } else {
            accumulatedTime = 0.0;
        }

        boolean timedStall = accumulatedTime >= ShooterConstants.HOOD_STALL_DETECTION_TIME_SEC;
        if (timedStall && ShooterConstants.HOOD_STALL_AUTO_STOP_ENABLED) {
            resetHoodMotorPosition();
        }
    }

    public boolean isHoodStalled() {
        return hoodStallDetected;
    }

    public boolean isHoodStallLatched() {
        return hoodStallLatched;
    }

    public void clearHoodStallLatch() {
        hoodStallAccumulatedTimeSec = 0.0;
        hoodStallDetected = false;
        hoodStallLatched = false;
    }

    private void forceDashboardTargetsResetAfterBoot() {
        // Delay once after boot so dashboard-retained values are overridden by robot defaults.
        if (dashboardTargetsResetAfterBoot || Timer.getFPGATimestamp() < 1.0) {
            return;
        }
        SmartDashboard.putNumber(
                ShooterConstants.HOOD_TARGET_DASHBOARD_KEY, ShooterConstants.HOOD_TARGET_DASHBOARD_DEFAULT_ROTATIONS);
        SmartDashboard.putNumber(
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_KEY,
                ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_TAG_DISTANCE_METERS_DASHBOARD_KEY,
                ShooterConstants.TUNING_TAG_DISTANCE_METERS_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_KEY,
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_DEFAULT);
        SmartDashboard.putNumber(
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        dashboardTargetsResetAfterBoot = true;
    }

    private void updateTuningDashboard() {
        double tuningDistanceMeters = tuningDistanceSupplier.getAsDouble();
        tuningFarShoot = SmartDashboard.getNumber(ShooterConstants.TUNING_FAR_SHOT, 1);
        tuningFarShootDistance = SmartDashboard.getNumber(ShooterConstants.TUNING_FAR_SHOT_DISTANCE, 5);
        SmartDashboard.putNumber(ShooterConstants.TUNING_DISTANCE_METERS_DASHBOARD_KEY, tuningDistanceMeters);
        SmartDashboard.putNumber(ShooterConstants.TUNING_TAG_DISTANCE_METERS_DASHBOARD_KEY, tuningDistanceMeters);
        double tuningHoodTargetDeg = SmartDashboard.getNumber(
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_KEY,
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_DEFAULT);
        double tuningFlywheelTargetRps = SmartDashboard.getNumber(
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        double predictedHoodDeg = evaluateHoodDegrees();
        double predictedFlywheelRps = evaluateFlywheelRps(tuningDistanceMeters);

        SmartDashboard.putNumber(ShooterConstants.TUNING_CURRENT_HOOD_DEG_DASHBOARD_KEY, getHoodAngleDegrees());
        SmartDashboard.putNumber(ShooterConstants.TUNING_CURRENT_FLYWHEEL_RPS_DASHBOARD_KEY, getFlywheelVelocityRps());
        SmartDashboard.putNumber(ShooterConstants.TUNING_PREDICTED_HOOD_DEG_DASHBOARD_KEY, predictedHoodDeg);
        SmartDashboard.putNumber(ShooterConstants.TUNING_PREDICTED_FLYWHEEL_RPS_DASHBOARD_KEY, predictedFlywheelRps);
        SmartDashboard.putNumber(ShooterConstants.TUNING_MODEL_DISTANCE_METERS_DASHBOARD_KEY, tuningDistanceMeters);
        SmartDashboard.putString(
                ShooterConstants.TUNING_SAMPLE_ROW_DASHBOARD_KEY,
                formatSampleRow(tuningDistanceMeters, tuningHoodTargetDeg, tuningFlywheelTargetRps));

        Logger.recordOutput("Shooter/Tuning/DistanceMeters", tuningDistanceMeters);
        Logger.recordOutput("Shooter/Tuning/TagDistanceMeters", tuningDistanceMeters);
        Logger.recordOutput("Shooter/Tuning/HoodTargetDeg", tuningHoodTargetDeg);
        Logger.recordOutput("Shooter/Tuning/FlywheelTargetRps", tuningFlywheelTargetRps);
        Logger.recordOutput("Shooter/Tuning/PredictedHoodDeg", predictedHoodDeg);
        Logger.recordOutput("Shooter/Tuning/PredictedFlywheelRps", predictedFlywheelRps);
    }

    private void updateDashboardTuning() {
        double dashFlywheelP =
                SmartDashboard.getNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kP", flywheelPidP);
        double dashFlywheelI =
                SmartDashboard.getNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kI", flywheelPidI);
        double dashFlywheelD =
                SmartDashboard.getNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kD", flywheelPidD);
        double dashFlywheelV =
                SmartDashboard.getNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kV", flywheelPidV);
        if (dashFlywheelP != flywheelPidP
                || dashFlywheelI != flywheelPidI
                || dashFlywheelD != flywheelPidD
                || dashFlywheelV != flywheelPidV) {
            flywheelPidP = dashFlywheelP;
            flywheelPidI = dashFlywheelI;
            flywheelPidD = dashFlywheelD;
            flywheelPidV = dashFlywheelV;
            io.setFlywheelPid(flywheelPidP, flywheelPidI, flywheelPidD, flywheelPidV);
        }

        double dashHoodP = SmartDashboard.getNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kP", hoodPidP);
        double dashHoodI = SmartDashboard.getNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kI", hoodPidI);
        double dashHoodD = SmartDashboard.getNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kD", hoodPidD);
        if (dashHoodP != hoodPidP || dashHoodI != hoodPidI || dashHoodD != hoodPidD) {
            hoodPidP = dashHoodP;
            hoodPidI = dashHoodI;
            hoodPidD = dashHoodD;
            io.setHoodPid(hoodPidP, hoodPidI, hoodPidD);
        }
    }

    public void runFlywheelVelocity(double rps) {
        if (!Double.isFinite(rps) || rps <= 0.0) {
            stopFlywheel();
            return;
        }

        if (!Double.isFinite(flywheelSetpointRps)
                || Math.abs(rps - flywheelSetpointRps) > ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS) {
            resetFlywheelReadiness();
        }
        controlMode = ControlMode.FLYWHEEL_VELOCITY;
        flywheelSetpointRps = rps;
        io.setFlywheelVelocityRps(rps);
    }

    public void runFlywheelDefaultSpeed() {
        runFlywheelVelocity(ShooterConstants.FLYWHEEL_DEFAULT_RPS);
    }

    public void runFlywheelDutyCycle(double output) {
        controlMode = ControlMode.FLYWHEEL_DUTY;
        flywheelSetpointRps = 0.0;
        resetFlywheelReadiness();
        io.setFlywheelDutyCycle(output);
    }

    public void runHoodDutyCycle(double output) {
        double clamped =
                MathUtil.clamp(output, -ShooterConstants.HOOD_TEST_MAX_DUTY, ShooterConstants.HOOD_TEST_MAX_DUTY);
        if (controlMode != ControlMode.HOOD_DUTY || Math.abs(clamped - hoodDutySetpoint) > 1e-3) {
            clearHoodStallLatch();
        }
        controlMode = ControlMode.HOOD_DUTY;
        hoodDutySetpoint = clamped;
        io.setHoodDutyCycle(clamped);
    }

    public void setHoodPositionRotations(double rotations) {
        if (!Double.isFinite(rotations)) {
            stopHood();
            return;
        }

        double clampedTarget = clampHoodRotations(rotations);
        if (controlMode != ControlMode.HOOD_POSITION || Math.abs(clampedTarget - hoodSetpointRotations) > 0.01) {
            clearHoodStallLatch();
        }
        controlMode = ControlMode.HOOD_POSITION;
        hoodSetpointRotations = clampedTarget;
        hoodDutySetpoint = 0.0;
        io.setHoodPositionRotations(hoodSetpointRotations);
    }

    public void setHoodAngleDegrees(double angleDeg) {
        setHoodPositionRotations(hoodDegreesToMotorRotations(angleDeg));
    }

    public void runShot(double hoodAngleDeg, double flywheelRps) {
        if (!Double.isFinite(hoodAngleDeg) || !Double.isFinite(flywheelRps) || flywheelRps <= 0.0) {
            stopFlywheel();
            return;
        }

        runFlywheelVelocity(flywheelRps);
        setHoodAngleDegrees(hoodAngleDeg);
    }

    public void runShotFromDistance() {
        runShot(evaluateHoodDegrees(), evaluateFlywheelRps(tuningDistanceSupplier.getAsDouble()));
    }

    public void runShotFromDistanceToHub(Translation2d target) {
        runShot(evaluateHoodDegreesHub(target), evaluateFlywheelRpsHub(target) - 1.5);
    }

    public void runShotFromDistanceMetersToHub(double distanceMeters) {
        runShot(evaluateHoodDegreesHub(distanceMeters), evaluateFlywheelRpsHub(distanceMeters) - 1.5);
    }

    public void runShuttleShot(Translation2d target) {
        setHoodAngleDegrees(1000);
        runFlywheelVelocity(evaluateFlywheelRpsHub(target));
    }

    public void stopFlywheel() {
        controlMode = ControlMode.IDLE;
        flywheelSetpointRps = 0.0;
        resetFlywheelReadiness();
        io.stopFlywheel();
    }

    public void stopHood() {
        if (controlMode == ControlMode.HOOD_DUTY || controlMode == ControlMode.HOOD_POSITION) {
            controlMode = ControlMode.IDLE;
        }
        hoodDutySetpoint = 0.0;
        hoodSetpointRotations = inputs.hoodPositionRotations;
        io.stopHood();
    }

    public void stopAll() {
        controlMode = ControlMode.IDLE;
        flywheelSetpointRps = 0.0;
        resetFlywheelReadiness();
        hoodDutySetpoint = 0.0;
        hoodSetpointRotations = inputs.hoodPositionRotations;
        io.stopAll();
    }

    public boolean atFlywheelSpeed(double targetRps, double toleranceRps) {
        return areFlywheelsWithinTolerance(
                inputs.flywheelConnected,
                inputs.flywheelLeaderVelocityRps,
                inputs.flywheelFollowerVelocityRps,
                targetRps,
                toleranceRps);
    }

    public boolean isFlywheelAtSetpoint() {
        boolean withinTolerance = Double.isFinite(flywheelSetpointRps)
                && flywheelSetpointRps > 0.0
                && atFlywheelSpeed(flywheelSetpointRps, ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS);
        return flywheelReadyDebouncer.calculate(withinTolerance);
    }

    static boolean areFlywheelsWithinTolerance(
            boolean connected, double leaderRps, double followerRps, double targetRps, double toleranceRps) {
        if (!connected
                || !Double.isFinite(leaderRps)
                || !Double.isFinite(followerRps)
                || !Double.isFinite(targetRps)
                || !Double.isFinite(toleranceRps)
                || toleranceRps < 0.0) {
            return false;
        }

        return Math.abs(leaderRps - targetRps) <= toleranceRps && Math.abs(followerRps - targetRps) <= toleranceRps;
    }

    private void resetFlywheelReadiness() {
        flywheelReadyDebouncer.calculate(false);
    }

    public double getFlywheelVelocityRps() {
        return inputs.flywheelVelocityRps;
    }

    public boolean atHoodPositionRotations(double targetRotations, double toleranceRotations) {
        return Math.abs(inputs.hoodPositionRotations - targetRotations) <= toleranceRotations;
    }

    public double getHoodPositionRotations() {
        return inputs.hoodPositionRotations;
    }

    public double getHoodAngleDegrees() {
        return motorRotationsToHoodDegrees(inputs.hoodPositionRotations);
    }

    public static double hoodDegreesToMotorRotations(double angleDeg) {
        double hoodRotations = (angleDeg - ShooterConstants.HOOD_ZERO_ANGLE_DEGREES) / 360.0;
        return hoodRotations * ShooterConstants.HOOD_MOTOR_ROTATIONS_PER_HOOD_ROTATION;
    }

    public static double clampHoodRotations(double rotations) {
        return MathUtil.clamp(rotations, 0.0, ShooterConstants.HOOD_MAX_ROTATIONS);
    }

    public static double motorRotationsToHoodDegrees(double motorRotations) {
        double hoodRotations = motorRotations / ShooterConstants.HOOD_MOTOR_ROTATIONS_PER_HOOD_ROTATION;
        return hoodRotations * 360.0 + ShooterConstants.HOOD_ZERO_ANGLE_DEGREES;
    }

    public double evaluateHoodDegrees() {
        return 0;
        // return 3.931 * Math.pow(10, -15) * Math.pow(distance, 49.95);
        // return ShooterConstants.HOOD_DISTANCE_SLOPE_DEG_PER_METER * distance
        //         + ShooterConstants.HOOD_DISTANCE_INTERCEPT_DEG;
    }

    public double evaluateFlywheelRps(double distanceMeters) {
        return 15 * distanceMeters + 32.35565;
        // return ShooterConstants.FLYWHEEL_DISTANCE_SLOPE_RPS_PER_METER * distanceMeters
        //         + ShooterConstants.FLYWHEEL_DISTANCE_INTERCEPT_RPS;
    }

    public double evaluateFlywheelRps() {
        double distance = tuningDistanceSupplier.getAsDouble();
        return evaluateFlywheelRps(distance);
    }
    // return ShooterConstants.FLYWHEEL_DISTANCE_SLOPE_RPS_PER_METER * distanceMeters
    //         + ShooterConstants.FLYWHEEL_DISTANCE_INTERCEPT_RPS;

    public double evaluateFlywheelRpsHub(Translation2d target) {
        return evaluateFlywheelRpsHub(drive.distanceToTargetMeters(target));
    }

    public double evaluateFlywheelRpsHub(double distanceMeters) {
        if (distanceMeters <= 2.5) {
            return 48;
        }

        if (distanceMeters >= tuningFarShootDistance) {
            return 0.0441314 * Math.pow(distanceMeters, 2)
                    + 3.44913 * distanceMeters
                    + 40.60355
                    + rpsOffset
                    + tuningFarShoot;
        }
        return 0.0441314 * Math.pow(distanceMeters, 2) + 3.44913 * distanceMeters + 40.60355 + rpsOffset;
    }

    public double evaluateHoodDegreesHub(Translation2d target) {
        return evaluateHoodDegreesHub(drive.distanceToTargetMeters(target));
    }

    public double evaluateHoodDegreesHub(double distanceMeters) {
        if (distanceMeters <= 2.5) {
            return 0;
        }
        return 217.3913 * distanceMeters - 326.08696;
    }

    private static String formatSampleRow(double distanceMeters, double hoodTargetDeg, double flywheelTargetRps) {
        return String.format(Locale.US, "%.3f,%.3f,%.3f", distanceMeters, hoodTargetDeg, flywheelTargetRps);
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void resetHoodMotorPosition() {
        io.resetHoodPosition(0);
        controlMode = ControlMode.HOOD_POSITION;
    }
}

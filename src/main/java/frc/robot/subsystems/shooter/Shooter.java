package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kP", flywheelPidP);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kI", flywheelPidI);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kD", flywheelPidD);
        SmartDashboard.putNumber(ShooterConstants.FLYWHEEL_PID_DASHBOARD_PREFIX + "kV", flywheelPidV);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kP", hoodPidP);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kI", hoodPidI);
        SmartDashboard.putNumber(ShooterConstants.HOOD_PID_DASHBOARD_PREFIX + "kD", hoodPidD);
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
        if (flywheelSetpointRps == 0.0) {
            resetHoodMotorPosition();
        }
        SmartDashboard.putNumber("Shooter/rpsOffset", rpsOffset);
        forceDashboardTargetsResetAfterBoot();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        updateDashboardTuning();

        Logger.recordOutput("Shooter/ControlMode", controlMode.toString());
        Logger.recordOutput("Shooter/FlywheelSetpointRps", flywheelSetpointRps);
        Logger.recordOutput("Shooter/Flywheel/TargetRps", flywheelSetpointRps);
        Logger.recordOutput("Shooter/Flywheel/CurrentRps", inputs.flywheelVelocityRps);
        Logger.recordOutput(
                "Shooter/FlywheelAtSpeed",
                atFlywheelSpeed(flywheelSetpointRps, ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS));
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
        SmartDashboard.putNumber("Shooter/Hood/CurrentDeg", getHoodAngleDegrees());
        // SmartDashboard.putNumber("Turret/TagDistance", tuningDistanceSupplier.getAsDouble());
        SmartDashboard.putNumber("", 0);
        updateTuningDashboard();
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
        io.setFlywheelDutyCycle(output);
    }

    public void runHoodDutyCycle(double output) {
        controlMode = ControlMode.HOOD_DUTY;
        double clamped =
                MathUtil.clamp(output, -ShooterConstants.HOOD_TEST_MAX_DUTY, ShooterConstants.HOOD_TEST_MAX_DUTY);
        io.setHoodDutyCycle(clamped);
    }

    public void setHoodPositionRotations(double rotations) {
        controlMode = ControlMode.HOOD_POSITION;
        hoodSetpointRotations = clampHoodRotations(rotations);
        io.setHoodPositionRotations(hoodSetpointRotations);
    }

    public void setHoodAngleDegrees(double angleDeg) {
        setHoodPositionRotations(hoodDegreesToMotorRotations(angleDeg));
    }

    public void runShot(double hoodAngleDeg, double flywheelRps) {
        runFlywheelVelocity(flywheelRps);
        setHoodAngleDegrees(hoodAngleDeg);
    }

    public void runShotFromDistance() {
        runShot(evaluateHoodDegrees(), evaluateFlywheelRps(tuningDistanceSupplier.getAsDouble()));
    }

    public void runShotFromDistanceToHub(Translation2d target) {
        runShot(evaluateHoodDegreesHub(target), evaluateFlywheelRpsHub(target));
    }

    public void runShuttleShot(Translation2d target) {
        setHoodAngleDegrees(1000);
        runFlywheelVelocity(evaluateFlywheelRpsHub(target));
    }

    public void stopFlywheel() {
        controlMode = ControlMode.IDLE;
        flywheelSetpointRps = 0.0;
        io.stopFlywheel();
    }

    public void stopHood() {
        if (controlMode == ControlMode.HOOD_DUTY || controlMode == ControlMode.HOOD_POSITION) {
            controlMode = ControlMode.IDLE;
        }
        hoodSetpointRotations = inputs.hoodPositionRotations;
        io.stopHood();
    }

    public void stopAll() {
        controlMode = ControlMode.IDLE;
        flywheelSetpointRps = 0.0;
        hoodSetpointRotations = inputs.hoodPositionRotations;
        io.stopAll();
    }

    public boolean atFlywheelSpeed(double targetRps, double toleranceRps) {
        return Math.abs(inputs.flywheelVelocityRps - targetRps) <= toleranceRps;
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
        double distance = tuningDistanceSupplier.getAsDouble();
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
        double distance = drive.distanceToTargetMeters(target);
        if (distance <= 2.5) {
            return 48;
        }

        return 0.0441314 * Math.pow(distance, 2) + 3.44913 * distance + 40.60355 + rpsOffset;
    }

    public double evaluateHoodDegreesHub(Translation2d target) {
        double distance = drive.distanceToTargetMeters(target);
        if (distance <= 2.5) {
            return 0;
        }
        return 217.3913 * distance - 326.08696;
    }

    private static String formatSampleRow(double distanceMeters, double hoodTargetDeg, double flywheelTargetRps) {
        return String.format(Locale.US, "%.3f,%.3f,%.3f", distanceMeters, hoodTargetDeg, flywheelTargetRps);
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void resetHoodMotorPosition() {
        hoodSetpointRotations = hoodDegreesToMotorRotations(ShooterConstants.HOOD_ZERO_ANGLE_DEGREES);
        io.setHoodPositionRotations(hoodSetpointRotations);
        controlMode = ControlMode.HOOD_POSITION;
    }
}

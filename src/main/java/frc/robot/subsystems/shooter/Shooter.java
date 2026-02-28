package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

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

    private ControlMode controlMode = ControlMode.IDLE;
    private double flywheelSetpointRps = 0.0;
    private double hoodSetpointRotations = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/ControlMode", controlMode.toString());
        Logger.recordOutput("Shooter/FlywheelSetpointRps", flywheelSetpointRps);
        Logger.recordOutput(
                "Shooter/FlywheelAtSpeed",
                atFlywheelSpeed(flywheelSetpointRps, ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS));
        Logger.recordOutput("Shooter/HoodSetpointRotations", hoodSetpointRotations);
        Logger.recordOutput(
                "Shooter/HoodAtTarget",
                atHoodPositionRotations(hoodSetpointRotations, ShooterConstants.HOOD_POSITION_TOLERANCE_ROTATIONS));
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
        hoodSetpointRotations = rotations;
        io.setHoodPositionRotations(rotations);
    }

    public void setHoodAngleDegrees(double angleDeg) {
        double hoodRotations = (angleDeg - ShooterConstants.HOOD_ZERO_ANGLE_DEGREES) / 360.0;
        double motorRotations = hoodRotations * ShooterConstants.HOOD_MOTOR_ROTATIONS_PER_HOOD_ROTATION;
        setHoodPositionRotations(motorRotations);
    }

    public void stopFlywheel() {
        controlMode = ControlMode.IDLE;
        flywheelSetpointRps = 0.0;
        io.stopFlywheel();
    }

    public void stopHood() {
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

    public ControlMode getControlMode() {
        return controlMode;
    }
}

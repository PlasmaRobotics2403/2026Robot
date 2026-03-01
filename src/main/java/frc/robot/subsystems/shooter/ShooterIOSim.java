package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2), 0.004, 1.0), DCMotor.getKrakenX60Foc(2));

    private double flywheelAppliedVolts = 0.0;
    private double hoodDutyCycle = 0.0;
    private double hoodPositionRotations = 0.0;
    private double hoodVelocityRps = 0.0;
    private double hoodPositionTargetRotations = 0.0;
    private boolean hoodPositionControlEnabled = false;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.update(0.02);

        inputs.flywheelConnected = true;
        inputs.flywheelVelocityRps = flywheelSim.getAngularVelocityRPM() / 60.0;
        inputs.flywheelLeaderVelocityRps = inputs.flywheelVelocityRps;
        inputs.flywheelFollowerVelocityRps = inputs.flywheelVelocityRps;
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.flywheelSupplyCurrentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.flywheelStatorCurrentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.flywheelTempCelsius = 0.0;
        inputs.flywheelClosedLoopErrorRps = 0.0;

        inputs.hoodConnected = true;
        if (hoodPositionControlEnabled) {
            double maxDelta = 0.04;
            double error = hoodPositionTargetRotations - hoodPositionRotations;
            double delta = MathUtil.clamp(error, -maxDelta, maxDelta);
            hoodPositionRotations += delta;
            hoodVelocityRps = delta / 0.02;
        } else {
            hoodVelocityRps = hoodDutyCycle;
            hoodPositionRotations += hoodVelocityRps * 0.02;
        }

        inputs.hoodPositionRotations = hoodPositionRotations;
        inputs.hoodVelocityRps = hoodVelocityRps;
        inputs.hoodAppliedVolts = hoodDutyCycle * 12.0;
        inputs.hoodCurrentAmps = 0.0;
        inputs.hoodTempCelsius = 0.0;
        inputs.hoodClosedLoopErrorRotations =
                hoodPositionControlEnabled ? (hoodPositionTargetRotations - hoodPositionRotations) : 0.0;
    }

    @Override
    public void setFlywheelVelocityRps(double rps) {
        flywheelAppliedVolts = MathUtil.clamp(rps * 0.12, -12.0, 12.0);
        flywheelSim.setInputVoltage(flywheelAppliedVolts);
    }

    @Override
    public void setFlywheelDutyCycle(double output) {
        flywheelAppliedVolts = MathUtil.clamp(output * 12.0, -12.0, 12.0);
        flywheelSim.setInputVoltage(flywheelAppliedVolts);
    }

    @Override
    public void setHoodDutyCycle(double output) {
        hoodPositionControlEnabled = false;
        hoodDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
    }

    @Override
    public void setHoodPositionRotations(double rotations) {
        hoodPositionControlEnabled = true;
        hoodPositionTargetRotations = rotations;
        hoodDutyCycle = 0.0;
    }

    @Override
    public void stopFlywheel() {
        flywheelAppliedVolts = 0.0;
        flywheelSim.setInputVoltage(0.0);
    }

    @Override
    public void stopHood() {
        hoodDutyCycle = 0.0;
        hoodPositionControlEnabled = false;
        hoodPositionTargetRotations = hoodPositionRotations;
    }
}

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public boolean flywheelConnected = false;
        public double flywheelVelocityRps = 0.0;
        public double flywheelLeaderVelocityRps = 0.0;
        public double flywheelFollowerVelocityRps = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelSupplyCurrentAmps = 0.0;
        public double flywheelStatorCurrentAmps = 0.0;
        public double flywheelTempCelsius = 0.0;
        public double flywheelClosedLoopErrorRps = 0.0;

        public boolean hoodConnected = false;
        public double hoodPositionRotations = 0.0;
        public double hoodVelocityRps = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;
        public double hoodTempCelsius = 0.0;
        public double hoodClosedLoopErrorRotations = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelVelocityRps(double rps) {}

    public default void setFlywheelDutyCycle(double output) {}

    public default void setHoodDutyCycle(double output) {}

    public default void setHoodPositionRotations(double rotations) {}

    public default void setFlywheelPid(double kP, double kI, double kD, double kV) {}

    public default void setHoodPid(double kP, double kI, double kD) {}

    public default void stopFlywheel() {}

    public default void stopHood() {}

    public default void stopAll() {
        stopFlywheel();
        stopHood();
    }
}

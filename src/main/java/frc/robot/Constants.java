package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.RobotDeviceId;

public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final boolean tuningMode = false;

    public static class RobotDevices {
        public static final class IntakeConstants {
            private IntakeConstants() {}

            public static final RobotDeviceId INTAKE_PIVOT = new RobotDeviceId(21, "rio", 10);
            public static final RobotDeviceId INTAKE_ROLLER = new RobotDeviceId(22, "rio", 11);
        }
    }

    public static final class IntakeConstants {
        public static final double DEPLOY_DEG = 95.0;
        public static final double STOW_DEG = 0.0;
        public static final double ROLLER_PERCENT = 0.5;
    }

    public static final class TurretConstants {
        public static final double kP = 4;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kCruiseVelocityRps = 200.0;
        public static final double kAccelerationRpsPerSec = 400.0;
        public static final double kJerkRpsPerSec2 = 4000.0;
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}

        public static final int FLYWHEEL_LEADER_CAN_ID = 30;
        public static final int FLYWHEEL_FOLLOWER_CAN_ID = 32;
        public static final int HOOD_CAN_ID = 31;
        public static final String CANBUS_NAME = "rio";

        public static final boolean FLYWHEEL_LEADER_INVERTED = false;
        public static final boolean FLYWHEEL_FOLLOWER_INVERTED = true;
        public static final boolean HOOD_INVERTED = true;

        public static final double FLYWHEEL_STATOR_CURRENT_LIMIT = 80.0;
        public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double HOOD_SUPPLY_CURRENT_LIMIT = 30.0;

        public static final double FLYWHEEL_KP = 0.5;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.0;
        public static final double FLYWHEEL_KV = 0.12; // volts per rps
        public static final double FLYWHEEL_KA = 0.0;
        public static final String FLYWHEEL_PID_DASHBOARD_PREFIX = "Shooter/Flywheel/PID/";

        public static final double FLYWHEEL_DEFAULT_RPS = 60.0;
        public static final double FLYWHEEL_SPEED_TOLERANCE_RPS = 2.0;
        public static final String FLYWHEEL_TARGET_RPS_DASHBOARD_KEY = "Shooter/Flywheel/TargetRps";
        public static final double FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT = FLYWHEEL_DEFAULT_RPS;

        public static final double HOOD_KP = 8.0;
        public static final double HOOD_KI = 0.0;
        public static final double HOOD_KD = 0.0;
        public static final double HOOD_KS = 0.0;
        public static final double HOOD_KV = 0.0;
        public static final double HOOD_KA = 0.0;
        public static final String HOOD_PID_DASHBOARD_PREFIX = "Shooter/Hood/PID/";
        public static final double HOOD_POSITION_TOLERANCE_ROTATIONS = 0.01;
        public static final double HOOD_MOTOR_ROTATIONS_PER_HOOD_ROTATION = 1.0;
        public static final double HOOD_ZERO_ANGLE_DEGREES = 0.0;
        public static final String HOOD_TARGET_DASHBOARD_KEY = "Shooter/Hood/TargetRotations";
        public static final double HOOD_TARGET_DASHBOARD_DEFAULT_ROTATIONS = 0.0;
        public static final String FLYWHEEL_DUTY_DASHBOARD_KEY = "Shooter/Flywheel/TargetDuty";
        public static final double FLYWHEEL_DUTY_DASHBOARD_DEFAULT = 0.40;

        public static final double SPINDEXER_FEED_DUTY = 0.50;
        public static final double SHOOTER_KICKER_FEED_DUTY = 0.50;

        public static final double FLYWHEEL_TEST_MAX_DUTY = 0.50;
        public static final double HOOD_TEST_MAX_DUTY = 0.25;
    }

    public static enum Mode {
        REAL,

        SIM,

        REPLAY
    }
}

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.RobotDeviceId;

public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final boolean tuningMode = false;
    public static final double blueHubX = 4.621; // meters
    public static final double blueHubY = 4; // meters
    public static final double redHubX = 11.893; // meters
    public static final double redHubY = 4; // meters

    public static final double blueFarShuttleX = 3.3; // meters
    public static final double blueFarShuttleY = 7.2; // meters

    public static final double blueNearShuttleX = 3.3; // meters
    public static final double blueNearShuttleY = 2.7; // meters

    public static final double redFarShuttleX = 13.4; // meters
    public static final double redFarShuttleY = 5.2; // meters

    public static final double redNearShuttleX = 13.4; // meters
    public static final double redNearShuttleY = 4; // meters

    public static class RobotDevices {
        public static final class IntakeConstants {
            private IntakeConstants() {}

            public static final RobotDeviceId INTAKE_PIVOT = new RobotDeviceId(21, "rio", 10);
            public static final RobotDeviceId INTAKE_ROLLER = new RobotDeviceId(22, "rio", 11);
            public static final RobotDeviceId INTAKE_ROLLER2 = new RobotDeviceId(50, "rio", 11);
        }
    }

    public static final class IntakeConstants {
        public static final double DEPLOY_DEG = 102.0;
        public static final double STOW_DEG = 0.0;
        public static final double ROLLER_PERCENT = 0.75;
    }

    public static final class TurretConstants {
        private TurretConstants() {}

        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kCruiseVelocityRps = 200.0;
        public static final double kAccelerationRpsPerSec = 300.0;
        public static final double kJerkRpsPerSec2 = 4000.0;

        public static final double MIN_ANGLE_DEG = -183.0;
        public static final double MAX_ANGLE_DEG = 183.0;
        public static final Translation2d TURRET_PIVOT_FROM_ROBOT_CENTER =
                new Translation2d(Units.inchesToMeters(-6.25), Units.inchesToMeters(6.25)); // 6.25

        // Camera calibration reference supplied by the user at turret angle -90 degrees (facing robot rear).
        // We model the camera as rigidly attached to the turret with a fixed offset from the turret pivot.
        public static final Transform3d TURRET_TO_CAMERA = new Transform3d(
                Units.inchesToMeters(-11.628 - (-6.25)),
                Units.inchesToMeters(-8.4846 - (-6.25)),
                Units.inchesToMeters(19.6099),
                new Rotation3d(0.0, Units.degreesToRadians(70.0), 0.0));

        public static final double TAG_LOCK_ENTER_DEBOUNCE_SEC = 0.05; // 0.05
        public static final double TAG_LOCK_EXIT_DEBOUNCE_SEC = 0.15; // 0.15

        // Placeholder shuttle/feed field point until dedicated interpolation map is added.
        public static final Translation2d DEFAULT_FEED_FIELD_POINT =
                new Translation2d(Units.inchesToMeters(325.0), Units.inchesToMeters(120.0));
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

        public static final double FLYWHEEL_KP = 0.65;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.03;
        public static final double FLYWHEEL_KS = 0.0;
        public static final double FLYWHEEL_KV = 0.1; // volts per rps
        public static final double FLYWHEEL_KA = 0.0;
        public static final String FLYWHEEL_PID_DASHBOARD_PREFIX = "Shooter/Flywheel/PID/";

        public static final double FLYWHEEL_DEFAULT_RPS = 55.0;
        public static final double FLYWHEEL_SPEED_TOLERANCE_RPS = 2.0;
        public static final String FLYWHEEL_TARGET_RPS_DASHBOARD_KEY = "Shooter/Flywheel/TargetRps";
        public static final double FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT = FLYWHEEL_DEFAULT_RPS;
        public static final String TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_KEY = "Shooter/Tuning/FlywheelTargetRps";
        public static final double TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT = FLYWHEEL_DEFAULT_RPS;

        public static final double HOOD_KP = 4.0;
        public static final double HOOD_KI = 0.0;
        public static final double HOOD_KD = 0.0;
        public static final double HOOD_KS = 0.0;
        public static final double HOOD_KV = 0.0;
        public static final double HOOD_KA = 0.0;
        public static final String HOOD_PID_DASHBOARD_PREFIX = "Shooter/Hood/PID/";
        public static final double HOOD_POSITION_TOLERANCE_ROTATIONS = 0.01;
        public static final double HOOD_MOTOR_ROTATIONS_PER_HOOD_ROTATION = 1.0;
        public static final double HOOD_ZERO_ANGLE_DEGREES = 0.0;
        public static final double HOOD_MAX_ROTATIONS = 5;
        public static final String HOOD_TARGET_DASHBOARD_KEY = "Shooter/Hood/TargetRotations";
        public static final double HOOD_TARGET_DASHBOARD_DEFAULT_ROTATIONS = 0.0;
        public static final String HOOD_TARGET_DEGREES_DASHBOARD_KEY = "Shooter/Hood/TargetDeg";
        public static final double HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT = 0.0;
        public static final String FLYWHEEL_DUTY_DASHBOARD_KEY = "Shooter/Flywheel/TargetDuty";
        public static final double FLYWHEEL_DUTY_DASHBOARD_DEFAULT = 0.40;
        public static final String TUNING_DISTANCE_METERS_DASHBOARD_KEY = "Shooter/Tuning/DistanceMeters";
        public static final double TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT = 2.0;
        public static final String TUNING_TAG_DISTANCE_METERS_DASHBOARD_KEY = "Shooter/Tuning/TagDistanceMeters";
        public static final double TUNING_TAG_DISTANCE_METERS_DASHBOARD_DEFAULT =
                TUNING_DISTANCE_METERS_DASHBOARD_DEFAULT;
        public static final String TUNING_HOOD_TARGET_DEG_DASHBOARD_KEY = "Shooter/Tuning/HoodTargetDeg";
        public static final double TUNING_HOOD_TARGET_DEG_DASHBOARD_DEFAULT = HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT;
        public static final String TUNING_CURRENT_HOOD_DEG_DASHBOARD_KEY = "Shooter/Tuning/CurrentHoodDeg";
        public static final String TUNING_CURRENT_FLYWHEEL_RPS_DASHBOARD_KEY = "Shooter/Tuning/CurrentFlywheelRps";
        public static final String TUNING_PREDICTED_HOOD_DEG_DASHBOARD_KEY = "Shooter/Tuning/PredictedHoodDeg";
        public static final String TUNING_PREDICTED_FLYWHEEL_RPS_DASHBOARD_KEY = "Shooter/Tuning/PredictedFlywheelRps";
        public static final String TUNING_SAMPLE_ROW_DASHBOARD_KEY = "Shooter/Tuning/SampleRow";
        public static final String TUNING_MODEL_DISTANCE_METERS_DASHBOARD_KEY = "Shooter/Tuning/ModelDistanceMeters";

        public static final double HOOD_DISTANCE_SLOPE_DEG_PER_METER = 0.0;
        public static final double HOOD_DISTANCE_INTERCEPT_DEG = 0.0;
        public static final double FLYWHEEL_DISTANCE_SLOPE_RPS_PER_METER = 0.0;
        public static final double FLYWHEEL_DISTANCE_INTERCEPT_RPS = FLYWHEEL_DEFAULT_RPS;

        public static final double SPINDEXER_FEED_DUTY = 0.50;
        public static final double SHOOTER_KICKER_FEED_DUTY = 0.6;

        public static final double FLYWHEEL_TEST_MAX_DUTY = 0.50;
        public static final double HOOD_TEST_MAX_DUTY = 0.25;
    }

    public static final class ClimbConstants {
        private ClimbConstants() {}

        public static final int MOTOR_CAN_ID = 35;
        public static final String CANBUS_NAME = "rio";

        public static final boolean MOTOR_INVERTED = false;
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLED = true;
        public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
        public static final boolean STATOR_CURRENT_LIMIT_ENABLED = false;

        public static final double POSITION_KP = 8.0;
        public static final double POSITION_KI = 0.0;
        public static final double POSITION_KD = 0.0;
        public static final double POSITION_KS = 0.0;
        public static final double POSITION_KV = 0.2;
        public static final double POSITION_KA = 0.0;

        public static final double POSITION_TOLERANCE_ROTATIONS = 0.05;
        public static final double TARGET_POSITION_ROTATIONS = 50.0;
        public static final double HOME_POSITION_ROTATIONS = 0.0;
        public static final double MAX_DUTY_CYCLE = 1.0;
    }

    public static enum Mode {
        REAL,

        SIM,

        REPLAY
    }
}

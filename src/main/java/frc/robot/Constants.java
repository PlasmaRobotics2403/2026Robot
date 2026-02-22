// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.RobotDeviceId;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final boolean tuningMode = false;

    /** Device IDs and power channels for added mechanism subsystems. */
    public static class RobotDevices {
        public static final class IntakeConstants {
            private IntakeConstants() {}

            public static final RobotDeviceId INTAKE_PIVOT = new RobotDeviceId(21, "rio", 10);
            public static final RobotDeviceId INTAKE_ROLLER = new RobotDeviceId(22, "rio", 11);
        }
    }

    /** Turret PID and motion magic tuning constants. */
    public static final class TurretConstants {
        public static final double kP = 6.7;
        public static final double kI = 0.0;
        public static final double kD = 0.00001;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kCruiseVelocityRps = 200.0;
        public static final double kAccelerationRpsPerSec = 400.0;
        public static final double kJerkRpsPerSec2 = 4000.0;
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}

package frc.robot.util;

import com.ctre.phoenix6.CANBus;

public class RobotDeviceId {
    private final int m_CANDeviceNumber;
    private final String m_CANBus;
    private final Integer m_PowerPort;

    public RobotDeviceId(int CANdeviceNumber, String CANbus, Integer powerPort) {
        m_CANDeviceNumber = CANdeviceNumber;
        m_CANBus = CANbus;
        m_PowerPort = powerPort;
    }

    public RobotDeviceId(int CANdeviceNumber, Integer powerPort) {
        this(CANdeviceNumber, "", powerPort);
    }

    public int getDeviceNumber() {
        return m_CANDeviceNumber;
    }

    public String getBus() {
        return m_CANBus;
    }

    public CANBus getCANBus() {
        return new CANBus(m_CANBus);
    }

    public int getPowerPort() {
        return m_PowerPort;
    }

    public boolean equals(RobotDeviceId other) {
        return other.m_CANDeviceNumber == m_CANDeviceNumber && other.m_CANBus == m_CANBus;
    }
}

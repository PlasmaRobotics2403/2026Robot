package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public TestMotorSubsystem(int canId) {
        this(canId, "rio");
    }

    public TestMotorSubsystem(int canId, String canBus) {
        motor = new TalonFX(canId, new CANBus(canBus));
    }

    public void setDutyCycle(double dutyCycle) {
        double clamped = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        motor.setControl(dutyCycleRequest.withOutput(clamped));
    }

    public void stop() {
        setDutyCycle(0.0);
    }
}

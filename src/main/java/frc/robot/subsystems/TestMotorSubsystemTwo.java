package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.RBSISubsystem;

/** Simple CAN motor test subsystem (duty-cycle control). */
public class TestMotorSubsystemTwo extends RBSISubsystem {

  private final TalonFX motor;
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  /** Create a test motor on the default CAN bus ("rio"). */
  public TestMotorSubsystemTwo(int canId) {
    this(canId, "rio");
  }

  /** Create a test motor on a specific CAN bus (e.g., "rio", "canivore"). */
  public TestMotorSubsystemTwo(int canId, String canBus) {
    motor = new TalonFX(canId, canBus);
  }

  /**
   * Sets the motor duty-cycle output.
   *
   * @param dutyCycle range [-1.0, 1.0]
   */
  public void setDutyCycle(double dutyCycle) {
    double clamped = MathUtil.clamp(dutyCycle, -1.0, 1.0);
    motor.setControl(dutyCycleRequest.withOutput(clamped));
  }

  public void stop() {
    setDutyCycle(0.0);
  }
}

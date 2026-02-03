package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestMotorSubsystem;
import java.util.function.DoubleSupplier;

public class TestMotorCommand extends Command {

  private final TestMotorSubsystem motor;
  private final DoubleSupplier dutyCycleSupplier;

  public TestMotorCommand(TestMotorSubsystem motor, double dutyCycle) {
    this(motor, () -> dutyCycle);
  }

  public TestMotorCommand(TestMotorSubsystem motor, DoubleSupplier dutyCycleSupplier) {
    this.motor = motor;
    this.dutyCycleSupplier = dutyCycleSupplier;
    addRequirements(motor);
  }

  @Override
  public void execute() {
    motor.setDutyCycle(dutyCycleSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    motor.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

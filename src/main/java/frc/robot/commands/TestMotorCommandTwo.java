package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestMotorSubsystemTwo;
import java.util.function.DoubleSupplier;

public class TestMotorCommandTwo extends Command {

    private final TestMotorSubsystemTwo motor;
    private final DoubleSupplier dutyCycleSupplier;

    public TestMotorCommandTwo(TestMotorSubsystemTwo motor, double dutyCycle) {
        this(motor, () -> dutyCycle);
    }

    public TestMotorCommandTwo(TestMotorSubsystemTwo motor, DoubleSupplier dutyCycleSupplier) {
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

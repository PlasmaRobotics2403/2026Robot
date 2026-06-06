package frc.robot.commands.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

public class TestTurretToPosCommandStatic extends Command {
    private final TestTurretSubsystem turret;

    public TestTurretToPosCommandStatic(TestTurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTargetAngleRadians(Units.degreesToRadians(-90));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // When the A button is released, command the turret back to zero and keep holding.
        // targetAngleRad = kZeroAngleRad;
        // targetAngleLimiter.reset(turret.getPositionRadians());
        // turret.setTargetAngleRadians(kZeroAngleRad);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

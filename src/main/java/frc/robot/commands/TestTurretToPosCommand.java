package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

public class TestTurretToPosCommand extends Command {
    private double kTestAngle;

    private static final double kZeroAngleRad = 0.0;

    private static final double kMaxAngleRad = Units.degreesToRadians(180.0);
    private static final double kSlewRateRadPerSec = Units.degreesToRadians(360.0);

    private final TestTurretSubsystem turret;

    private final SlewRateLimiter targetAngleLimiter = new SlewRateLimiter(kSlewRateRadPerSec);
    private double targetAngleRad = 0.0;

    public TestTurretToPosCommand(TestTurretSubsystem turret, double pos) {
        this.turret = turret;
        kTestAngle = Units.degreesToRadians(pos);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        targetAngleRad = kTestAngle;
        targetAngleLimiter.reset(turret.getPositionRadians());
        turret.setTargetAngleRadians(turret.getPositionRadians());
    }

    @Override
    public void execute() {
        double limitedTargetRad = targetAngleLimiter.calculate(targetAngleRad);
        limitedTargetRad = Math.min(Math.max(limitedTargetRad, -kMaxAngleRad), kMaxAngleRad);
        turret.setTargetAngleRadians(limitedTargetRad);
    }

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

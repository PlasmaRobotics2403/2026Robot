package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFromDistanceCommand extends Command {
    private final Shooter shooter;

    public ShootFromDistanceCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.runShotFromDistance();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

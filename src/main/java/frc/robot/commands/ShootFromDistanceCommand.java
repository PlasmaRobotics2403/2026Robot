package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShootFromDistanceCommand extends Command {
    private final Shooter shooter;
    private final DoubleSupplier distanceMetersSupplier;

    public ShootFromDistanceCommand(Shooter shooter, DoubleSupplier distanceMetersSupplier) {
        this.shooter = shooter;
        this.distanceMetersSupplier = distanceMetersSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.runShotFromDistance(distanceMetersSupplier.getAsDouble());
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

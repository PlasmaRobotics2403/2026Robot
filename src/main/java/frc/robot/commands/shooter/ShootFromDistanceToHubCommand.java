package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFromDistanceToHubCommand extends Command {
    private final Shooter shooter;
    private final TestTurretSubsystem turret;
    private final Drive drive;

    public ShootFromDistanceToHubCommand(Shooter shooter, TestTurretSubsystem turret, Drive drive) {
        this.shooter = shooter;
        this.turret = turret;
        this.drive = drive;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        HubShotController.updateShotSolution(shooter, turret, drive);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
        turret.clearUnwinding();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

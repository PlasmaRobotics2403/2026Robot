package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;

    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double flywheelTargetRps = SmartDashboard.getNumber(
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        shooter.runFlywheelVelocity(flywheelTargetRps);
        double hoodTargetRotations = SmartDashboard.getNumber(
                ShooterConstants.HOOD_TARGET_DASHBOARD_KEY, ShooterConstants.HOOD_TARGET_DASHBOARD_DEFAULT_ROTATIONS);
        shooter.setHoodPositionRotations(hoodTargetRotations);
        SmartDashboard.putNumber("Shooter/Hood/CurrentRotations", shooter.getHoodPositionRotations());
        SmartDashboard.putNumber("Shooter/Flywheel/CurrentRps", shooter.getFlywheelVelocityRps());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodPositionRotations(ShooterConstants.HOOD_TARGET_DASHBOARD_DEFAULT_ROTATIONS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

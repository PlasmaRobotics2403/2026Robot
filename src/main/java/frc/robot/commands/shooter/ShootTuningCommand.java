package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootTuningCommand extends Command {
    private final Shooter shooter;

    public ShootTuningCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double flywheelTargetRps = SmartDashboard.getNumber(
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.TUNING_FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        double hoodTargetDeg = SmartDashboard.getNumber(
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_KEY,
                ShooterConstants.TUNING_HOOD_TARGET_DEG_DASHBOARD_DEFAULT);
        shooter.runShot(hoodTargetDeg, flywheelTargetRps);
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

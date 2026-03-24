package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double hubX, hubY;
        if (alliance == Alliance.Blue) {
            hubX = Constants.blueHubX;
            hubY = Constants.blueHubY;
        } else {
            hubX = Constants.redHubX;
            hubY = Constants.redHubY;
        }
        Translation2d hubField = new Translation2d(hubX, hubY);
        double flywheelTargetRps = SmartDashboard.getNumber(
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_KEY,
                ShooterConstants.FLYWHEEL_TARGET_RPS_DASHBOARD_DEFAULT);
        shooter.runFlywheelVelocity(flywheelTargetRps);
        double hoodTargetDeg = SmartDashboard.getNumber(
                ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_KEY,
                ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
        shooter.setHoodAngleDegrees(shooter.evaluateHoodDegreesHub(hubField));
        SmartDashboard.putNumber("Shooter/Hood/CurrentRotations", shooter.getHoodPositionRotations());
        SmartDashboard.putNumber("Shooter/Flywheel/CurrentRps", shooter.getFlywheelVelocityRps());
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

public class TurretFlipCommand extends Command {
    private final TestTurretSubsystem turret;

    private static final double OPPOSITE_LIMIT_MARGIN_DEG = 25.0;
    private static final double FINISH_TOLERANCE_DEG = 8.0;
    private static final double TIMEOUT_SEC = 2.0;

    private double targetAngleRad = 0.0;
    private double startTimeSec = 0.0;

    public TurretFlipCommand(TestTurretSubsystem turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        startTimeSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        double currentAngleRad = turret.getPositionRadians();
        double marginRad = Math.toRadians(OPPOSITE_LIMIT_MARGIN_DEG);

        if (currentAngleRad >= 0.0) {
            targetAngleRad = turret.MIN_ANGLE_RAD + marginRad;
        } else {
            targetAngleRad = turret.MAX_ANGLE_RAD - marginRad;
        }

        turret.setTargetAngleRadians(targetAngleRad);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        double errRad = Math.abs(turret.getPositionRadians() - targetAngleRad);
        if (errRad <= Math.toRadians(FINISH_TOLERANCE_DEG)) {
            return true;
        }

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return (now - startTimeSec) >= TIMEOUT_SEC;
    }
}

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;

public class TestTurretFollowCommand extends Command {
    private static final double kSlewRateRadPerSec = Units.degreesToRadians(360.0);

    private final TestTurretSubsystem turret;
    private final Vision vision;
    private final int cameraIndex;

    private final SlewRateLimiter targetAngleLimiter = new SlewRateLimiter(kSlewRateRadPerSec);
    private double targetAngleRad = 0.0;

    private double lastFlipTimeSec = -1.0;
    private static final double kFlipCooldownSec = 1;

    private TurretFlipCommand activeFlipCommand;
    private boolean flipArmed = true;

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision, int cameraIndex) {
        this.turret = turret;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        addRequirements(turret);
    }

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision) {
        this(turret, vision, 0);
    }

    @Override
    public void initialize() {
        targetAngleRad = turret.getPositionRadians();
        targetAngleLimiter.reset(targetAngleRad);
        turret.setTargetAngleRadians(targetAngleRad);

        activeFlipCommand = null;
        lastFlipTimeSec = -1.0;
        flipArmed = true;
    }

    @Override
    public void execute() {
        Rotation2d tx = vision.getTargetX(cameraIndex);

        turret.setTargetAngleRadians(turret.getPositionRadians() - tx.getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

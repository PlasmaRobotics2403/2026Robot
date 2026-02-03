package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;

public class TestTurretFollowCommand extends Command {
  private static final int kTagId = 1;
  private static final double kMaxStepRadPerCycle = Units.degreesToRadians(5.0);
  private static final double kMaxAngleRad = Units.degreesToRadians(170.0);
  private static final double kDeadbandRad = Units.degreesToRadians(0.75);
  private static final double kSlewRateRadPerSec = Units.degreesToRadians(360.0);

  private final TestTurretSubsystem turret;
  private final Vision vision;
  private final int cameraIndex;

  private final SlewRateLimiter targetAngleLimiter = new SlewRateLimiter(kSlewRateRadPerSec);
  private double targetAngleRad = 0.0;

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
  }

  @Override
  public void execute() {
    boolean seesTag = vision.seesTag(cameraIndex, kTagId);
    Rotation2d tx = vision.getTargetXForTag(cameraIndex, kTagId);

    if (!seesTag) {
      targetAngleRad = turret.getPositionRadians();
      targetAngleLimiter.reset(targetAngleRad);
      turret.setTargetAngleRadians(targetAngleRad);
      return;
    }

    // double yawErrRad = MathUtil.applyDeadband(tx.getRadians(), kDeadbandRad);

    // double correctionRad = -yawErrRad;
    // correctionRad = MathUtil.clamp(correctionRad, -kMaxStepRadPerCycle, kMaxStepRadPerCycle);

    // targetAngleRad = MathUtil.clamp(targetAngleRad + correctionRad, -kMaxAngleRad, kMaxAngleRad);
    // targetAngleRad = targetAngleLimiter.calculate(targetAngleRad);

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

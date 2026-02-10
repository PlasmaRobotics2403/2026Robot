package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;

public class TestTurretFollowCommand extends Command {
  private static final int kTagId = 18;
  private static final double kMaxStepRadPerCycle = Units.degreesToRadians(5.0);
  private static final double kMaxAngleRad = Units.degreesToRadians(170.0);
  private static final double kDeadbandRad = Units.degreesToRadians(0.75);
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
    // If we're currently flipping/unwinding, don't overwrite the target with vision tracking.
    if (activeFlipCommand != null) {
      if (activeFlipCommand.isScheduled()) {
        return;
      }
      // Flip finished; clear it so we can resume tracking.
      activeFlipCommand = null;
    }

    // Re-arm once we've exited the limit band.
    // This prevents ping-ponging when the unwind ends near the opposite limit.
    if (!turret.isAtLimit()) {
      flipArmed = true;
    }

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
    if (flipArmed && turret.isAtLimit()) {
      double now = Timer.getFPGATimestamp();
      if (lastFlipTimeSec < 0.0 || (now - lastFlipTimeSec) > kFlipCooldownSec) {
        lastFlipTimeSec = now;
        activeFlipCommand = new TurretFlipCommand(turret);
        CommandScheduler.getInstance().schedule(activeFlipCommand);
        flipArmed = false;
      }
      return;
    }

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

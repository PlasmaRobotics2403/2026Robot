package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lightweight turret implementation for testing the state machine without touching real hardware.
 *
 * <p>This is not intended for competition use. It's a safe way to develop command logic while
 * keeping {@code TestTurretSubsystem} untouched.
 */
public class SimTurretSubsystem extends SubsystemBase implements TurretSubsystem {
  private final double minAngleRad;
  private final double maxAngleRad;
  private final double limitBandRad;

  private double positionRad = 0.0;
  private double targetRad = 0.0;
  private boolean enabled = false;

  // Very simple first-order motion model.
  private static final double MAX_SPEED_RAD_PER_SEC = Units.degreesToRadians(240.0);

  private double lastTimestampSec = Timer.getFPGATimestamp();

  public SimTurretSubsystem(double minAngleRad, double maxAngleRad) {
    this(minAngleRad, maxAngleRad, Units.degreesToRadians(6.0));
  }

  public SimTurretSubsystem(double minAngleRad, double maxAngleRad, double limitBandRad) {
    this.minAngleRad = minAngleRad;
    this.maxAngleRad = maxAngleRad;
    this.limitBandRad = limitBandRad;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestampSec;
    lastTimestampSec = now;

    if (!enabled) {
      return;
    }

    double error = targetRad - positionRad;
    double maxStep = MAX_SPEED_RAD_PER_SEC * dt;
    double step = MathUtil.clamp(error, -maxStep, maxStep);
    positionRad = MathUtil.clamp(positionRad + step, minAngleRad, maxAngleRad);
  }

  @Override
  public double getPositionRadians() {
    return positionRad;
  }

  @Override
  public boolean isAtLimit() {
    return positionRad <= (minAngleRad + limitBandRad)
        || positionRad >= (maxAngleRad - limitBandRad);
  }

  @Override
  public void setTargetAngle(Rotation2d angle) {
    targetRad = MathUtil.clamp(angle.getRadians(), minAngleRad, maxAngleRad);
    enabled = true;
  }

  @Override
  public void stop() {
    enabled = false;
  }

  @Override
  public double getMinAngleRad() {
    return minAngleRad;
  }

  @Override
  public double getMaxAngleRad() {
    return maxAngleRad;
  }
}

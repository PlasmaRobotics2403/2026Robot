package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Competition turret API.
 *
 * <p>This is intentionally separate from {@code TestTurretSubsystem} so the test versions can
 * remain as a fallback.
 */
public interface TurretSubsystem extends Subsystem {
  /** Returns the turret output angle (post-gearbox). */
  double getPositionRadians();

  /** Returns true when the turret is near either soft limit. */
  boolean isAtLimit();

  /** Sets a target angle (post-gearbox) and enables holding that position. */
  void setTargetAngle(Rotation2d angle);

  default void setTargetAngleRadians(double angleRad) {
    setTargetAngle(Rotation2d.fromRadians(angleRad));
  }

  /** Stops the turret. */
  void stop();

  /** Returns the minimum allowed angle (radians). */
  double getMinAngleRad();

  /** Returns the maximum allowed angle (radians). */
  double getMaxAngleRad();
}

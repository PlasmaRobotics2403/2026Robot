package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Small helper to throttle dashboard publishing.
 *
 * <p>Why: Pushing lots of NetworkTables/SmartDashboard keys at 50Hz can gradually bog down
 * dashboards (Elastic, Shuffleboard) during long driver-station sessions.
 *
 * <p>Usage:
 *
 * <pre>
 * if (DashboardThrottle.shouldPublish("Vision", 0.2)) {
 *   SmartDashboard.putNumber("Vision/CameraCount", io.length);
 * }
 * </pre>
 */
public final class DashboardThrottle {
  private DashboardThrottle() {}

  private static final ConcurrentHashMap<String, Double> lastPublishSecByKey =
      new ConcurrentHashMap<>();

  /**
   * Returns true if at least {@code periodSec} has elapsed since the last time this key was
   * allowed.
   */
  public static boolean shouldPublish(String key, double periodSec) {
    double now = Timer.getFPGATimestamp();

    // getOrDefault avoids an extra put on the first call.
    double last = lastPublishSecByKey.getOrDefault(key, Double.NEGATIVE_INFINITY);
    if ((now - last) < periodSec) {
      return false;
    }

    lastPublishSecByKey.put(key, now);
    return true;
  }

  /** Clears all throttle state (useful for tests or mode transitions). */
  public static void reset() {
    lastPublishSecByKey.clear();
  }
}

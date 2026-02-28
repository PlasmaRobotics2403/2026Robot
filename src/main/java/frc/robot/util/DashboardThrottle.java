package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.ConcurrentHashMap;

public final class DashboardThrottle {
    private DashboardThrottle() {}

    private static final ConcurrentHashMap<String, Double> lastPublishSecByKey = new ConcurrentHashMap<>();

    public static boolean shouldPublish(String key, double periodSec) {
        double now = Timer.getFPGATimestamp();

        double last = lastPublishSecByKey.getOrDefault(key, Double.NEGATIVE_INFINITY);
        if ((now - last) < periodSec) {
            return false;
        }

        lastPublishSecByKey.put(key, now);
        return true;
    }

    public static void reset() {
        lastPublishSecByKey.clear();
    }
}

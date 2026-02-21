package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DrivebaseConstants.BumpSimConstants;

/**
 * Simulation-only bump traversal shaping model for the 2026 field.
 *
 * <p>MapleSim drivetrain is 2D, so this approximates bump traversal by shaping commanded chassis
 * speeds while crossing known bump regions.
 */
public class BumpZoneModel {
  private static final double BLUE_BUMP_CENTER_X_M = 4.5974;
  private static final double RED_BUMP_CENTER_X_M = 11.9380;
  private static final double BUMP_CENTER_Y_M = 4.034536;

  // Bump collider dimensions used by Maple's 2026 arena when ramps are represented.
  private static final double BUMP_WIDTH_X_M = 1.1938; // 47 in
  private static final double BUMP_LENGTH_Y_M = 5.5118; // 217 in

  private static final double HALF_Y_SPLIT_MARGIN_M = 0.03;

  public enum Phase {
    NONE,
    ASCENT,
    CREST,
    DESCENT
  }

  public static final class Result {
    public final ChassisSpeeds speeds;
    public final boolean inZone;
    public final String zoneName;
    public final Phase phase;
    public final double progress;
    public final double speedScale;
    public final double headingErrorDeg;

    public Result(
        ChassisSpeeds speeds,
        boolean inZone,
        String zoneName,
        Phase phase,
        double progress,
        double speedScale,
        double headingErrorDeg) {
      this.speeds = speeds;
      this.inZone = inZone;
      this.zoneName = zoneName;
      this.phase = phase;
      this.progress = progress;
      this.speedScale = speedScale;
      this.headingErrorDeg = headingErrorDeg;
    }
  }

  private Rotation2d headingHoldTarget = Rotation2d.kZero;
  private boolean headingLocked = false;
  private String previousZone = "";
  private double lastVx = 0.0;
  private double lastVy = 0.0;

  public Result apply(ChassisSpeeds desired, Pose2d pose, double dtSec) {
    if (!BumpSimConstants.enabled || dtSec <= 0.0) {
      return new Result(desired, false, "", Phase.NONE, 0.0, 1.0, 0.0);
    }

    Zone zone = findZone(pose);
    if (zone == null) {
      headingLocked = false;
      previousZone = "";
      lastVx = desired.vxMetersPerSecond;
      lastVy = desired.vyMetersPerSecond;
      return new Result(desired, false, "", Phase.NONE, 0.0, 1.0, 0.0);
    }

    double planarSpeed = Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond);
    if (planarSpeed < BumpSimConstants.minCrossingSpeedMps) {
      return new Result(desired, true, zone.name, Phase.CREST, 0.5, 1.0, 0.0);
    }

    if (!zone.name.equals(previousZone)) {
      headingHoldTarget = pose.getRotation();
      headingLocked = true;
      previousZone = zone.name;
    }

    double direction = Math.signum(desired.vxMetersPerSecond);
    if (Math.abs(direction) < 1e-6) {
      direction = pose.getX() < zone.centerX ? 1.0 : -1.0;
    }

    double progress =
        direction > 0.0
            ? (pose.getX() - zone.minX) / (zone.maxX - zone.minX)
            : (zone.maxX - pose.getX()) / (zone.maxX - zone.minX);
    progress = MathUtil.clamp(progress, 0.0, 1.0);

    Phase phase;
    double speedScale;
    if (progress < 0.35) {
      phase = Phase.ASCENT;
      double t = smoothstep(progress / 0.35);
      speedScale = lerp(1.0, BumpSimConstants.entrySpeedScale, t);
    } else if (progress < 0.65) {
      phase = Phase.CREST;
      speedScale = BumpSimConstants.crestSpeedScale;
    } else {
      phase = Phase.DESCENT;
      double t = smoothstep((progress - 0.65) / 0.35);
      speedScale = lerp(BumpSimConstants.crestSpeedScale, 1.0, t);
    }

    double targetVx = desired.vxMetersPerSecond * speedScale;
    double targetVy = desired.vyMetersPerSecond * BumpSimConstants.lateralDamping;

    double maxDeltaV = BumpSimConstants.maxAccelMps2 * dtSec;
    double dvx = MathUtil.clamp(targetVx - lastVx, -maxDeltaV, maxDeltaV);
    double dvy = MathUtil.clamp(targetVy - lastVy, -maxDeltaV, maxDeltaV);
    double shapedVx = lastVx + dvx;
    double shapedVy = lastVy + dvy;
    lastVx = shapedVx;
    lastVy = shapedVy;

    double omega = desired.omegaRadiansPerSecond;
    double headingErrorDeg = 0.0;
    if (headingLocked) {
      double errRad =
          MathUtil.angleModulus(headingHoldTarget.minus(pose.getRotation()).getRadians());
      headingErrorDeg = Math.toDegrees(errRad);
      double correction =
          MathUtil.clamp(
              BumpSimConstants.headingHoldKp * errRad,
              -BumpSimConstants.headingHoldMaxOmegaRadPerSec,
              BumpSimConstants.headingHoldMaxOmegaRadPerSec);
      omega += correction;
    }

    return new Result(
        new ChassisSpeeds(shapedVx, shapedVy, omega),
        true,
        zone.name,
        phase,
        progress,
        speedScale,
        headingErrorDeg);
  }

  private static double smoothstep(double t) {
    t = MathUtil.clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static Zone findZone(Pose2d pose) {
    for (Zone z : ZONES) {
      if (pose.getX() >= z.minX
          && pose.getX() <= z.maxX
          && pose.getY() >= z.minY
          && pose.getY() <= z.maxY) {
        return z;
      }
    }
    return null;
  }

  private static final Zone[] ZONES = buildZones();

  private static Zone[] buildZones() {
    double transition = BumpSimConstants.transitionMeters;
    double halfWidthX = (BUMP_WIDTH_X_M / 2.0) + transition;
    double halfLenY = BUMP_LENGTH_Y_M / 2.0;

    double yMin = BUMP_CENTER_Y_M - halfLenY;
    double yMax = BUMP_CENTER_Y_M + halfLenY;
    double yMidLow = BUMP_CENTER_Y_M - HALF_Y_SPLIT_MARGIN_M;
    double yMidHigh = BUMP_CENTER_Y_M + HALF_Y_SPLIT_MARGIN_M;

    return new Zone[] {
      new Zone(
          "BlueSouth",
          BLUE_BUMP_CENTER_X_M,
          BLUE_BUMP_CENTER_X_M - halfWidthX,
          BLUE_BUMP_CENTER_X_M + halfWidthX,
          yMin,
          yMidLow),
      new Zone(
          "BlueNorth",
          BLUE_BUMP_CENTER_X_M,
          BLUE_BUMP_CENTER_X_M - halfWidthX,
          BLUE_BUMP_CENTER_X_M + halfWidthX,
          yMidHigh,
          yMax),
      new Zone(
          "RedSouth",
          RED_BUMP_CENTER_X_M,
          RED_BUMP_CENTER_X_M - halfWidthX,
          RED_BUMP_CENTER_X_M + halfWidthX,
          yMin,
          yMidLow),
      new Zone(
          "RedNorth",
          RED_BUMP_CENTER_X_M,
          RED_BUMP_CENTER_X_M - halfWidthX,
          RED_BUMP_CENTER_X_M + halfWidthX,
          yMidHigh,
          yMax)
    };
  }

  private static final class Zone {
    final String name;
    final double centerX;
    final double minX;
    final double maxX;
    final double minY;
    final double maxY;

    Zone(String name, double centerX, double minX, double maxX, double minY, double maxY) {
      this.name = name;
      this.centerX = centerX;
      this.minX = minX;
      this.maxX = maxX;
      this.minY = minY;
      this.maxY = maxY;
    }
  }
}

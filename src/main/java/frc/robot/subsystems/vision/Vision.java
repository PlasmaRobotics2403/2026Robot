// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024-2025 FRC 2486
// http://github.com/Coconuts2486-FRC
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.angularStdDevBaseline;
import static frc.robot.Constants.VisionConstants.angularStdDevMegatag2Factor;
import static frc.robot.Constants.VisionConstants.linearStdDevBaseline;
import static frc.robot.Constants.VisionConstants.linearStdDevMegatag2Factor;
import static frc.robot.Constants.VisionConstants.maxAmbiguity;
import static frc.robot.Constants.VisionConstants.maxZError;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.DashboardThrottle;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Dashboard outputs (AdvantageKit NT wrappers) so values reliably show up in this project.
  private final LoggedNetworkNumber camera0PrimaryTagId =
      new LoggedNetworkNumber("SmartDashboard/Vision/Camera0/PrimaryTagId", -1.0);
  private final LoggedNetworkNumber camera0Connected =
      new LoggedNetworkNumber("SmartDashboard/Vision/Camera0/Connected", 0.0);
  private final LoggedNetworkNumber camera0TagCount =
      new LoggedNetworkNumber("SmartDashboard/Vision/Camera0/TagCount", 0.0);

  private static final double K_DASHBOARD_PUBLISH_PERIOD_SEC = 0.2; // 5 Hz

  private double lastPoseArrayLogSec = 0.0;
  private static final double K_POSE_ARRAY_LOG_PERIOD_SEC = 0.5; // 2 Hz

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Log the robot-to-camera transformations
    Logger.recordOutput("Vision/RobotToCamera0", Constants.Cameras.robotToCamera0);
    Logger.recordOutput("Vision/RobotToCamera1", Constants.Cameras.robotToCamera1);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /** Returns true if the camera currently reports seeing the given AprilTag ID. */
  public boolean seesTag(int cameraIndex, int tagId) {
    for (int id : inputs[cameraIndex].tagIds) {
      if (id == tagId) {
        return true;
      }
    }
    return false;
  }

  /**
   * Returns a single "primary" detected AprilTag ID for the camera.
   *
   * <p>This is meant for quick debugging on dashboards. If multiple tags are visible, this returns
   * the first ID in the current inputs list (ordering is implementation-dependent). If no tags are
   * visible, returns -1.
   */
  public int getPrimaryTagId(int cameraIndex) {
    if (inputs[cameraIndex].tagIds.length == 0) {
      return -1;
    }
    return inputs[cameraIndex].tagIds[0];
  }

  /**
   * Returns the camera-to-tag yaw (tx) for a specific AprilTag ID.
   *
   * <p>If the tag is not present, returns {@link Rotation2d#kZero}.
   */
  public Rotation2d getTargetXForTag(int cameraIndex, int tagId) {
    for (int i = 0; i < inputs[cameraIndex].tagIds.length; i++) {
      if (inputs[cameraIndex].tagIds[i] == tagId) {
        if (i < inputs[cameraIndex].tagYaw.length) {
          return inputs[cameraIndex].tagYaw[i];
        }
        return Rotation2d.kZero;
      }
    }
    return Rotation2d.kZero;
  }

  @Override
  public void periodic() {
    // Heartbeat so we can tell at a glance that Vision.periodic() is executing.
    // Avoid spamming DriverStation warnings every second (it can bog things down over time).
    // Light dashboard fields (throttled)
    if (DashboardThrottle.shouldPublish("Vision/Dashboard", K_DASHBOARD_PUBLISH_PERIOD_SEC)) {
      SmartDashboard.putBoolean("Vision/Heartbeat", true);
      SmartDashboard.putNumber("Vision/CameraCount", io.length);
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);

      // Quick debug number: show one detected tag ID (or -1 if none)
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(i) + "/PrimaryTagId", getPrimaryTagId(i));
      // SmartDashboard publishing is throttled below.

      // Helpful extra debug fields
      if (DashboardThrottle.shouldPublish(
          "Vision/Camera" + Integer.toString(i) + "/Dashboard", K_DASHBOARD_PUBLISH_PERIOD_SEC)) {
        SmartDashboard.putNumber(
            "Vision/Camera" + Integer.toString(i) + "/PrimaryTagId", getPrimaryTagId(i));
        SmartDashboard.putBoolean(
            "Vision/Camera" + Integer.toString(i) + "/Connected", inputs[i].connected);
        SmartDashboard.putNumber(
            "Vision/Camera" + Integer.toString(i) + "/TagCount", inputs[i].tagIds.length);
      }

      // Also publish with LoggedNetworkNumber (camera 0 for now) so the value appears even if the
      // native SmartDashboard UI isn't auto-populating new keys.
      if (i == 0) {
        camera0PrimaryTagId.set(getPrimaryTagId(0));
        camera0Connected.set(inputs[0].connected ? 1.0 : 0.0);
        camera0TagCount.set(inputs[0].tagIds.length);
      }
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    double nowSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    boolean shouldLogPoseArrays = false;
    if (Constants.tuningMode) {
      if (nowSec - lastPoseArrayLogSec >= K_POSE_ARRAY_LOG_PERIOD_SEC) {
        shouldLogPoseArrays = true;
        lastPoseArrayLogSec = nowSec;
      }
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FieldConstants.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FieldConstants.aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < Constants.Cameras.cameraStdDevFactors.length) {
          linearStdDev *= Constants.Cameras.cameraStdDevFactors[cameraIndex];
          angularStdDev *= Constants.Cameras.cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Heavy pose arrays can bog down NT dashboards over time. Only log occasionally and only
      // during tuning.
      if (shouldLogPoseArrays) {
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      }
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data (same throttling as above)
    if (shouldLogPoseArrays) {
      Logger.recordOutput(
          "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesAccepted",
          allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesRejected",
          allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

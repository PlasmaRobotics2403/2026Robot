package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.cameraLocalizationEnabled;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public Rotation2d getTargetYaw(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.yaw();
    }

    public boolean hasAnyTarget(int cameraIndex) {
        return inputs[cameraIndex].tagIds.length > 0;
    }

    public Optional<Rotation2d> getTargetX(int cameraIndex, int tagId) {
        for (var targetObservation : inputs[cameraIndex].taggedTargetObservations) {
            if (targetObservation.tagId() == tagId) {
                return Optional.of(targetObservation.tx());
            }
        }
        return Optional.empty();
    }

    public Optional<Rotation2d> getTargetYaw(int cameraIndex, int tagId) {
        for (var targetObservation : inputs[cameraIndex].taggedTargetObservations) {
            if (targetObservation.tagId() == tagId) {
                return Optional.of(targetObservation.yaw());
            }
        }
        return Optional.empty();
    }

    public Optional<Rotation2d> getTargetX(int cameraIndex, int... tagIds) {
        Set<Integer> allowedIds = java.util.Arrays.stream(tagIds).boxed().collect(Collectors.toSet());
        Rotation2d bestTx = null;
        for (var targetObservation : inputs[cameraIndex].taggedTargetObservations) {
            if (!allowedIds.contains(targetObservation.tagId())) {
                continue;
            }
            if (bestTx == null || Math.abs(targetObservation.tx().getRadians()) < Math.abs(bestTx.getRadians())) {
                bestTx = targetObservation.tx();
            }
        }
        return Optional.ofNullable(bestTx);
    }

    public Optional<Rotation2d> getTargetYaw(int cameraIndex, int... tagIds) {
        Set<Integer> allowedIds = java.util.Arrays.stream(tagIds).boxed().collect(Collectors.toSet());
        Rotation2d bestYaw = null;
        for (var targetObservation : inputs[cameraIndex].taggedTargetObservations) {
            if (!allowedIds.contains(targetObservation.tagId())) {
                continue;
            }
            if (bestYaw == null || Math.abs(targetObservation.yaw().getRadians()) < Math.abs(bestYaw.getRadians())) {
                bestYaw = targetObservation.yaw();
            }
        }
        return Optional.ofNullable(bestYaw);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            boolean localizationEnabled =
                    cameraIndex >= cameraLocalizationEnabled.length || cameraLocalizationEnabled[cameraIndex];
            if (Constants.currentMode == Constants.Mode.SIM) {
                localizationEnabled = false;
            }
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/LocalizationEnabled", localizationEnabled);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose = observation.tagCount() == 0
                        || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                        || Math.abs(observation.pose().getZ()) > maxZError
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                if (rejectPose) {
                    continue;
                }

                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                if (localizationEnabled) {
                    consumer.accept(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                }
            }

            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(Pose3d[]::new));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(Pose3d[]::new));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}

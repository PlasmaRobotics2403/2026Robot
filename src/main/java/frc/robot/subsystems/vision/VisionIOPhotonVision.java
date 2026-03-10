package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        List<TaggedTargetObservation> taggedTargetObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()),
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()));
                taggedTargetObservations.clear();
                for (var target : result.targets) {
                    taggedTargetObservations.add(new TaggedTargetObservation(
                            target.fiducialId,
                            Rotation2d.fromDegrees(target.getYaw()),
                            Rotation2d.fromDegrees(target.getPitch()),
                            Rotation2d.fromDegrees(target.getYaw())));
                }
            } else {
                inputs.latestTargetObservation =
                        new TargetObservation(new Rotation2d(), new Rotation2d(), new Rotation2d());
                taggedTargetObservations.clear();
            }

            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance +=
                            target.bestCameraToTarget.getTranslation().getNorm();
                }

                tagIds.addAll(multitagResult.fiducialIDsUsed);

                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / result.targets.size(),
                        PoseObservationType.PHOTONVISION));

            } else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIds.add((short) target.fiducialId);

                    poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.getTranslation().getNorm(),
                            PoseObservationType.PHOTONVISION));
                }
            }
        }

        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        inputs.taggedTargetObservations = new TaggedTargetObservation[taggedTargetObservations.size()];
        for (int i = 0; i < taggedTargetObservations.size(); i++) {
            inputs.taggedTargetObservations[i] = taggedTargetObservations.get(i);
        }

        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}

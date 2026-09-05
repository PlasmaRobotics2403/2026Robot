package frc.robot.subsystems.drive;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Follows a PathPlanner path and uses PathPlanner's obstacle-aware pathfinder to recover from drift. */
public class ReplanningPathCommand extends Command {
    private static final double DRIFT_THRESHOLD_METERS = 0.60;
    private static final double RECOVERY_TIMEOUT_SECONDS = 2.50;
    private static final int MAX_RECOVERIES_PER_PATH = 2;
    private static final double FINAL_TRANSLATION_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double FINAL_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(1.0);
    private static final double FLOATING_POINT_EPSILON = 1e-9;

    private final PathPlannerPath originalPath;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Consumer<ChassisSpeeds> output;
    private final Supplier<PathFollowingController> controllerSupplier;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;

    private final Timer recoveryTimer = new Timer();
    private Command activeCommand;
    private PathPlannerPath activePath;
    private boolean recovering;
    private boolean finished;
    private boolean recoveryFailed;
    private int recoveryCount;

    public ReplanningPathCommand(
            PathPlannerPath path,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<ChassisSpeeds> output,
            Supplier<PathFollowingController> controllerSupplier,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Subsystem drive) {
        this.originalPath = path;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.output = output;
        this.controllerSupplier = controllerSupplier;
        this.robotConfig = robotConfig;
        this.shouldFlipPath = shouldFlipPath;

        addRequirements(drive);
        addRequirements(EventScheduler.getSchedulerRequirements(path));
    }

    @Override
    public void initialize() {
        activePath = orientPath(originalPath);
        recovering = false;
        finished = false;
        recoveryFailed = false;
        recoveryCount = 0;
        recoveryTimer.stop();
        Logger.recordOutput("PathPlanner/RecoveryActive", false);
        Logger.recordOutput("PathPlanner/RecoveryFailed", false);
        Logger.recordOutput("PathPlanner/NominalTimeElapsed", false);
        Logger.recordOutput("PathPlanner/FinalPoseWithinTolerance", false);
        Logger.recordOutput("PathPlanner/FinalTranslationErrorMeters", 0.0);
        Logger.recordOutput("PathPlanner/FinalRotationErrorDegrees", 0.0);
        startFollowPath(activePath);
    }

    @Override
    public void execute() {
        if (finished || activeCommand == null) {
            return;
        }

        activeCommand.execute();

        if (recovering && recoveryTimer.hasElapsed(RECOVERY_TIMEOUT_SECONDS)) {
            failRecovery();
            return;
        }

        if (activeCommand.isFinished()) {
            if (recovering) {
                activeCommand.end(false);
                recovering = false;
                recoveryTimer.stop();
                Logger.recordOutput("PathPlanner/RecoveryActive", false);
                startFollowPath(activePath);
            } else if (isAtFinalPose()) {
                activeCommand.end(false);
                activeCommand = null;
                finished = true;
            }
            return;
        }

        if (!recovering && shouldRecover()) {
            startRecovery();
        }
    }

    @Override
    public boolean isFinished() {
        return finished || recoveryFailed;
    }

    @Override
    public void end(boolean interrupted) {
        recoveryTimer.stop();

        if (activeCommand != null) {
            activeCommand.end(interrupted);
            activeCommand = null;
        }

        if (interrupted || recoveryFailed) {
            output.accept(new ChassisSpeeds());
        }

        Logger.recordOutput("PathPlanner/RecoveryActive", false);
        Logger.recordOutput("PathPlanner/NominalTimeElapsed", false);
    }

    private void startFollowPath(PathPlannerPath path) {
        Logger.recordOutput("PathPlanner/NominalTimeElapsed", false);
        activeCommand = new FollowPathCommand(
                path,
                poseSupplier,
                speedsSupplier,
                (speeds, feedforwards) -> output.accept(speeds),
                controllerSupplier.get(),
                robotConfig,
                () -> false,
                getRequirements().toArray(Subsystem[]::new));
        activeCommand.initialize();
    }

    private boolean isAtFinalPose() {
        Pose2d currentPose = poseSupplier.get();
        Pose2d targetPose = getFinalPose(activePath);
        double translationError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationErrorRadians = Math.abs(
                currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        boolean withinTolerance = isWithinFinalPoseTolerance(currentPose, targetPose);

        Logger.recordOutput("PathPlanner/NominalTimeElapsed", true);
        Logger.recordOutput("PathPlanner/FinalTargetPose", targetPose);
        Logger.recordOutput("PathPlanner/FinalTranslationErrorMeters", translationError);
        Logger.recordOutput("PathPlanner/FinalRotationErrorDegrees", Units.radiansToDegrees(rotationErrorRadians));
        Logger.recordOutput("PathPlanner/FinalPoseWithinTolerance", withinTolerance);
        return withinTolerance;
    }

    private static Pose2d getFinalPose(PathPlannerPath path) {
        List<PathPoint> pathPoints = path.getAllPathPoints();
        PathPoint finalPoint = pathPoints.get(pathPoints.size() - 1);
        return new Pose2d(finalPoint.position, path.getGoalEndState().rotation());
    }

    static boolean isWithinFinalPoseTolerance(Pose2d currentPose, Pose2d targetPose) {
        double translationError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(
                currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        return Double.isFinite(translationError)
                && Double.isFinite(rotationError)
                && translationError <= FINAL_TRANSLATION_TOLERANCE_METERS + FLOATING_POINT_EPSILON
                && rotationError <= FINAL_ROTATION_TOLERANCE_RADIANS + FLOATING_POINT_EPSILON;
    }

    private void startRecovery() {
        int targetWaypoint = findRecoveryWaypoint(activePath);
        if (targetWaypoint < 0 || recoveryCount >= MAX_RECOVERIES_PER_PATH) {
            return;
        }

        targetWaypoint = Math.min(targetWaypoint, activePath.getWaypoints().size() - 2);
        Pose2d targetPose = poseAtWaypoint(activePath, targetWaypoint);
        PathPlannerPath resumePath = createSuffixPath(activePath, targetWaypoint);
        if (resumePath == null) {
            return;
        }

        activeCommand.end(true);
        activePath = resumePath;
        activeCommand = new PathfindingCommand(
                targetPose,
                recoveryConstraints(activePath),
                poseSupplier,
                speedsSupplier,
                (speeds, feedforwards) -> output.accept(speeds),
                controllerSupplier.get(),
                robotConfig,
                getRequirements().toArray(Subsystem[]::new));
        activeCommand.initialize();

        recovering = true;
        recoveryCount++;
        recoveryTimer.reset();
        recoveryTimer.start();
        Logger.recordOutput("PathPlanner/RecoveryActive", true);
        Logger.recordOutput("PathPlanner/RecoveryTarget", targetPose);
        Logger.recordOutput("PathPlanner/RecoveryCount", recoveryCount);
    }

    private void failRecovery() {
        activeCommand.end(true);
        activeCommand = null;
        recovering = false;
        recoveryFailed = true;
        recoveryTimer.stop();
        output.accept(new ChassisSpeeds());
        Logger.recordOutput("PathPlanner/RecoveryActive", false);
        Logger.recordOutput("PathPlanner/RecoveryFailed", true);
    }

    private boolean shouldRecover() {
        if (recoveryCount >= MAX_RECOVERIES_PER_PATH || activePath == null) {
            return false;
        }

        double closestDistance = activePath.getAllPathPoints().stream()
                .mapToDouble(
                        point -> point.position.getDistance(poseSupplier.get().getTranslation()))
                .min()
                .orElse(Double.POSITIVE_INFINITY);
        return closestDistance > DRIFT_THRESHOLD_METERS;
    }

    private int findRecoveryWaypoint(PathPlannerPath path) {
        List<PathPoint> points = path.getAllPathPoints();
        if (points.isEmpty() || path.getWaypoints().size() < 2) {
            return -1;
        }

        PathPoint nearestPoint = points.get(0);
        double nearestDistance =
                nearestPoint.position.getDistance(poseSupplier.get().getTranslation());
        for (PathPoint point : points) {
            double distance = point.position.getDistance(poseSupplier.get().getTranslation());
            if (distance < nearestDistance) {
                nearestPoint = point;
                nearestDistance = distance;
            }
        }

        int waypointIndex = (int) Math.ceil(nearestPoint.waypointRelativePos);
        waypointIndex = Math.max(1, waypointIndex);
        return Math.min(waypointIndex, path.getWaypoints().size() - 1);
    }

    private Pose2d poseAtWaypoint(PathPlannerPath path, int waypointIndex) {
        Waypoint waypoint = path.getWaypoints().get(waypointIndex);
        Pose2d closestPose = new Pose2d(waypoint.anchor(), Rotation2d.kZero);
        double closestDistance = Double.POSITIVE_INFINITY;

        for (Pose2d pose : path.getPathPoses()) {
            double distance = pose.getTranslation().getDistance(waypoint.anchor());
            if (distance < closestDistance) {
                closestPose = pose;
                closestDistance = distance;
            }
        }

        return closestPose;
    }

    private PathPlannerPath createSuffixPath(PathPlannerPath path, int startWaypointIndex) {
        List<Waypoint> waypoints = path.getWaypoints();
        if (startWaypointIndex < 0 || startWaypointIndex >= waypoints.size()) {
            return null;
        }

        int lastWaypointIndex = waypoints.size() - 1;
        List<Waypoint> suffixWaypoints = new ArrayList<>(waypoints.subList(startWaypointIndex, waypoints.size()));
        List<RotationTarget> rotationTargets = new ArrayList<>();
        for (RotationTarget target : path.getRotationTargets()) {
            if (target.position() >= startWaypointIndex && target.position() <= lastWaypointIndex) {
                rotationTargets.add(new RotationTarget(target.position() - startWaypointIndex, target.rotation()));
            }
        }

        List<PointTowardsZone> pointTowardsZones = new ArrayList<>();
        for (PointTowardsZone zone : path.getPointTowardsZones()) {
            if (zone.maxPosition() >= startWaypointIndex && zone.minPosition() <= lastWaypointIndex) {
                pointTowardsZones.add(new PointTowardsZone(
                        zone.name(),
                        zone.targetPosition(),
                        zone.rotationOffset(),
                        Math.max(0.0, zone.minPosition() - startWaypointIndex),
                        Math.min(lastWaypointIndex - startWaypointIndex, zone.maxPosition() - startWaypointIndex)));
            }
        }

        List<ConstraintsZone> constraintZones = new ArrayList<>();
        for (ConstraintsZone zone : path.getConstraintZones()) {
            if (zone.maxPosition() >= startWaypointIndex && zone.minPosition() <= lastWaypointIndex) {
                constraintZones.add(new ConstraintsZone(
                        Math.max(0.0, zone.minPosition() - startWaypointIndex),
                        Math.min(lastWaypointIndex - startWaypointIndex, zone.maxPosition() - startWaypointIndex),
                        zone.constraints()));
            }
        }

        List<EventMarker> eventMarkers = new ArrayList<>();
        for (EventMarker marker : path.getEventMarkers()) {
            double sourceEndPosition = marker.endPosition() < 0.0 ? marker.position() : marker.endPosition();
            if (sourceEndPosition < startWaypointIndex) {
                continue;
            }

            double position = Math.max(0.0, marker.position() - startWaypointIndex);
            double endPosition = Math.max(position, sourceEndPosition - startWaypointIndex);
            endPosition = Math.min(lastWaypointIndex - startWaypointIndex, endPosition);
            if (marker.command() == null) {
                eventMarkers.add(new EventMarker(marker.triggerName(), position, endPosition));
            } else {
                eventMarkers.add(new EventMarker(marker.triggerName(), position, endPosition, marker.command()));
            }
        }

        PathPlannerPath suffix = new PathPlannerPath(
                suffixWaypoints,
                rotationTargets,
                pointTowardsZones,
                constraintZones,
                eventMarkers,
                path.getGlobalConstraints(),
                null,
                path.getGoalEndState(),
                path.isReversed());
        suffix.preventFlipping = true;
        suffix.name = path.name + " Recovery";
        return suffix;
    }

    private PathConstraints recoveryConstraints(PathPlannerPath path) {
        PathConstraints constraints = path.getGlobalConstraints();
        return new PathConstraints(
                Math.min(constraints.maxVelocityMPS(), 2.0),
                Math.min(constraints.maxAccelerationMPSSq(), 2.0),
                constraints.maxAngularVelocityRadPerSec(),
                constraints.maxAngularAccelerationRadPerSecSq(),
                constraints.nominalVoltageVolts(),
                constraints.unlimited());
    }

    private PathPlannerPath orientPath(PathPlannerPath path) {
        PathPlannerPath orientedPath = shouldFlipPath.getAsBoolean() ? path.flipPath() : path;
        orientedPath.preventFlipping = true;
        return orientedPath;
    }
}

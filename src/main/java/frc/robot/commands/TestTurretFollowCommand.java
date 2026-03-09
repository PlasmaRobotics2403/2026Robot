package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TurretGridSelector.GridTarget;
import frc.robot.util.TurretGridSelector.GridZone;
import frc.robot.util.TurretTargetingUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TestTurretFollowCommand extends Command {
    private static final String kFollowOffsetDegKey = "Turret/Follow/OffsetDeg";

    private final TestTurretSubsystem turret;
    private final Vision vision;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final int cameraIndex;
    private Optional<GridZone> previousZone = Optional.empty();

    public TestTurretFollowCommand(
            TestTurretSubsystem turret, Vision vision, Supplier<Pose2d> robotPoseSupplier, int cameraIndex) {
        this.turret = turret;
        this.vision = vision;
        this.robotPoseSupplier = robotPoseSupplier;
        this.cameraIndex = cameraIndex;
        addRequirements(turret);
    }

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision, Supplier<Pose2d> robotPoseSupplier) {
        this(turret, vision, robotPoseSupplier, 0);
    }

    @Override
    public void initialize() {
        previousZone = Optional.empty();
        turret.setTargetAngleRadians(turret.getPositionRadians());
        SmartDashboard.putNumber(kFollowOffsetDegKey, SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0));
        SmartDashboard.putNumber("Turret/Follow/TxDeg", 0.0);
    }

    @Override
    public void execute() {
        double followOffsetDeg = SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0);
        double followOffsetRad = Units.degreesToRadians(followOffsetDeg);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d robotPose = robotPoseSupplier.get();
        TurretTargetingUtil.FollowSolution followSolution = TurretTargetingUtil.calculateFollowSolution(
                robotPose, alliance, previousZone, vision, cameraIndex, followOffsetRad);
        GridTarget gridTarget = followSolution.gridTarget();
        previousZone = Optional.of(gridTarget.zone());
        Pose2d targetPose2d = gridTarget.targetPose().toPose2d();
        double txTrimRad = followSolution.txTrimRad();
        double requestedTargetAngleRad = MathUtil.angleModulus(followSolution.targetAngleRad());

        TurretTargetingUtil.FlipPlan flipPlan = TurretTargetingUtil.planFlip(
                turret.getPositionRadians(), requestedTargetAngleRad, turret.MIN_ANGLE_RAD, turret.MAX_ANGLE_RAD);
        boolean unwindActive = flipPlan.shouldUnwind();
        double preClampCommandAngleRad = unwindActive
                ? flipPlan.unwindTargetAngleRad().orElse(requestedTargetAngleRad)
                : requestedTargetAngleRad;
        double commandedAngleRad = MathUtil.clamp(preClampCommandAngleRad, turret.MIN_ANGLE_RAD, turret.MAX_ANGLE_RAD);
        boolean saturated = Math.abs(preClampCommandAngleRad - commandedAngleRad) > 1e-6;

        Logger.recordOutput("Turret/Follow/OffsetDeg", followOffsetDeg);
        Logger.recordOutput("Turret/Follow/Alliance", alliance.toString());
        Logger.recordOutput("Turret/Follow/Zone", gridTarget.zone().toString());
        Logger.recordOutput("Turret/Follow/TargetType", "TAG");
        Logger.recordOutput("Turret/Follow/PrimaryTagId", gridTarget.primaryTagId());
        Logger.recordOutput("Turret/Follow/TrimTagIds", gridTarget.trimTagIds());
        Logger.recordOutput("Turret/Follow/TargetPointX", targetPose2d.getX());
        Logger.recordOutput("Turret/Follow/TargetPointY", targetPose2d.getY());
        Logger.recordOutput(
                "Turret/Follow/PoseAimDeg", followSolution.poseAimAngle().getDegrees());
        Logger.recordOutput(
                "Turret/Follow/TrimTagSeen", followSolution.txForTarget().isPresent());
        Logger.recordOutput(
                "Turret/Follow/TargetTagSeen", followSolution.txForTarget().isPresent());
        Logger.recordOutput("Turret/Follow/UsingSpecificTag", true);
        Logger.recordOutput("Turret/Follow/TxDeg", Units.radiansToDegrees(txTrimRad));
        Logger.recordOutput("Turret/Follow/RequestedTargetDeg", Units.radiansToDegrees(requestedTargetAngleRad));
        Logger.recordOutput("Turret/Follow/PreClampCommandedAngleDeg", Units.radiansToDegrees(preClampCommandAngleRad));
        Logger.recordOutput("Turret/Follow/CommandedAngleDeg", Units.radiansToDegrees(commandedAngleRad));
        Logger.recordOutput("Turret/Follow/UnwindActive", unwindActive);
        Logger.recordOutput(
                "Turret/Follow/UnwindTargetDeg",
                flipPlan.unwindTargetAngleRad().isPresent()
                        ? Units.radiansToDegrees(flipPlan.unwindTargetAngleRad().getAsDouble())
                        : Double.NaN);
        Logger.recordOutput("Turret/Follow/Saturated", saturated);
        Logger.recordOutput("Turret/Follow/AtLimit", turret.isAtLimit());

        SmartDashboard.putString("Turret/Follow/Alliance", alliance.toString());
        SmartDashboard.putString("Turret/Follow/Zone", gridTarget.zone().toString());
        SmartDashboard.putString("Turret/Follow/TargetType", "TAG");
        SmartDashboard.putNumber("Turret/Follow/PrimaryTagId", gridTarget.primaryTagId());
        SmartDashboard.putNumber("Turret/Follow/TargetPointX", targetPose2d.getX());
        SmartDashboard.putNumber("Turret/Follow/TargetPointY", targetPose2d.getY());
        SmartDashboard.putNumber("Turret/Follow/TxDeg", Units.radiansToDegrees(txTrimRad));
        SmartDashboard.putBoolean(
                "Turret/Follow/TrimTagSeen", followSolution.txForTarget().isPresent());
        SmartDashboard.putBoolean("Turret/Follow/UnwindActive", unwindActive);
        SmartDashboard.putBoolean("Turret/Follow/Saturated", saturated);
        SmartDashboard.putBoolean("Turret/Follow/AtLimit", turret.isAtLimit());
        SmartDashboard.putNumber("Turret/Follow/RequestedTargetDeg", Units.radiansToDegrees(requestedTargetAngleRad));
        SmartDashboard.putNumber("Turret/Follow/CommandedAngleDeg", Units.radiansToDegrees(commandedAngleRad));

        turret.setTargetAngleRadians(commandedAngleRad);
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

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TurretGridSelector.GridZone;
import frc.robot.util.TurretTargetingUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretFlipCommand extends Command {
    private static final String kFollowOffsetDegKey = "Turret/Follow/OffsetDeg";

    private final TestTurretSubsystem turret;
    private final Vision vision;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final int cameraIndex;

    private static final double FINISH_TOLERANCE_DEG = 8.0;
    private static final double TIMEOUT_SEC = 2.5;

    private enum FlipStage {
        UNWIND,
        FINAL
    }

    private FlipStage stage = FlipStage.FINAL;
    private Optional<GridZone> previousZone = Optional.empty();
    private double requestedFlipTargetRad = 0.0;
    private double unwindTargetRad = 0.0;
    private double finalTargetRad = 0.0;
    private double startTimeSec = 0.0;

    public TurretFlipCommand(
            TestTurretSubsystem turret, Vision vision, Supplier<Pose2d> robotPoseSupplier, int cameraIndex) {
        this.turret = turret;
        this.vision = vision;
        this.robotPoseSupplier = robotPoseSupplier;
        this.cameraIndex = cameraIndex;
        addRequirements(turret);
    }

    public TurretFlipCommand(TestTurretSubsystem turret, Vision vision, Supplier<Pose2d> robotPoseSupplier) {
        this(turret, vision, robotPoseSupplier, 0);
    }

    @Override
    public void initialize() {
        startTimeSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        previousZone = Optional.empty();

        double followOffsetDeg = SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0);
        double followOffsetRad = Units.degreesToRadians(followOffsetDeg);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        TurretTargetingUtil.FollowSolution followSolution = TurretTargetingUtil.calculateFollowSolution(
                robotPoseSupplier.get(), alliance, previousZone, vision, cameraIndex, followOffsetRad);
        previousZone = Optional.of(followSolution.gridTarget().zone());

        TurretTargetingUtil.FlipPlan flipPlan = TurretTargetingUtil.planFlip(
                turret.getPositionRadians(),
                followSolution.targetAngleRad(),
                turret.MIN_ANGLE_RAD,
                turret.MAX_ANGLE_RAD);

        requestedFlipTargetRad = flipPlan.requestedFlipAngleRad();
        finalTargetRad = flipPlan.finalTargetAngleRad();
        double currentAngleRad = turret.getPositionRadians();

        if (flipPlan.shouldUnwind()) {
            stage = FlipStage.UNWIND;
            unwindTargetRad = flipPlan.unwindTargetAngleRad().orElse(currentAngleRad);
            turret.setTargetAngleRadians(unwindTargetRad);
        } else {
            stage = FlipStage.FINAL;
            unwindTargetRad = currentAngleRad;
            turret.setTargetAngleRadians(finalTargetRad);
        }

        Logger.recordOutput("Turret/Flip/GridFollowTargetDeg", Units.radiansToDegrees(followSolution.targetAngleRad()));
        Logger.recordOutput("Turret/Flip/RequestedTargetDeg", Units.radiansToDegrees(requestedFlipTargetRad));
        Logger.recordOutput("Turret/Flip/ChosenBranch", flipPlan.chosenBranch().toString());
        Logger.recordOutput("Turret/Flip/UseUnwind", flipPlan.shouldUnwind());
        Logger.recordOutput(
                "Turret/Flip/UnwindTargetDeg",
                flipPlan.unwindTargetAngleRad().isPresent()
                        ? Units.radiansToDegrees(flipPlan.unwindTargetAngleRad().getAsDouble())
                        : Double.NaN);
        Logger.recordOutput("Turret/Flip/FinalTargetDeg", Units.radiansToDegrees(finalTargetRad));
    }

    @Override
    public void execute() {
        if (stage == FlipStage.UNWIND && isAtTarget(unwindTargetRad)) {
            stage = FlipStage.FINAL;
            turret.setTargetAngleRadians(finalTargetRad);
        }

        Logger.recordOutput("Turret/Flip/Stage", stage.toString());
        Logger.recordOutput(
                "Turret/Flip/CommandedTargetDeg",
                Units.radiansToDegrees(stage == FlipStage.UNWIND ? unwindTargetRad : finalTargetRad));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        if (stage == FlipStage.FINAL && isAtTarget(finalTargetRad)) {
            return true;
        }

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return (now - startTimeSec) >= TIMEOUT_SEC;
    }

    private boolean isAtTarget(double targetAngleRad) {
        double errRad = Math.abs(turret.getPositionRadians() - targetAngleRad);
        return errRad <= Math.toRadians(FINISH_TOLERANCE_DEG);
    }
}

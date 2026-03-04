package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class TestTurretFollowCommand extends Command {
    private static final String kFollowOffsetDegKey = "Turret/Follow/OffsetDeg";

    private final TestTurretSubsystem turret;
    private final Vision vision;
    private final int cameraIndex;
    private final int targetTagId;

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision, int cameraIndex, int targetTagId) {
        this.turret = turret;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.targetTagId = targetTagId;
        addRequirements(turret);
    }

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision, int cameraIndex) {
        this(turret, vision, cameraIndex, -1);
    }

    public TestTurretFollowCommand(TestTurretSubsystem turret, Vision vision) {
        this(turret, vision, 0, -1);
    }

    @Override
    public void initialize() {
        turret.setTargetAngleRadians(turret.getPositionRadians());
        SmartDashboard.putNumber(kFollowOffsetDegKey, SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0));
        SmartDashboard.putNumber("Turret/Follow/TxDeg", 0.0);
    }

    @Override
    public void execute() {
        double followOffsetDeg = SmartDashboard.getNumber(kFollowOffsetDegKey, 0.0);
        double followOffsetRad = Units.degreesToRadians(followOffsetDeg);
        Logger.recordOutput("Turret/Follow/OffsetDeg", followOffsetDeg);

        // if (targetTagId > 0) {
        //     var txForTag = vision.getTargetX(cameraIndex, targetTagId);
        //     Logger.recordOutput("Turret/Follow/UsingSpecificTag", true);
        //     Logger.recordOutput("Turret/Follow/TargetTagId", targetTagId);
        //     Logger.recordOutput("Turret/Follow/TargetTagSeen", txForTag.isPresent());
        //     if (txForTag.isPresent()) {
        //         Rotation2d tx = txForTag.get();
        //         Logger.recordOutput("Turret/Follow/TxDeg", tx.getDegrees());
        //         SmartDashboard.putNumber("Turret/Follow/TxDeg", tx.getDegrees());
        //         turret.setTargetAngleRadians(turret.getPositionRadians() + tx.getRadians() - followOffsetRad);
        //     } else {
        //         SmartDashboard.putNumber("Turret/Follow/TxDeg", 0.0);
        //         turret.setTargetAngleRadians(turret.getPositionRadians());
        //     }
        //     return;
        // }

        Logger.recordOutput("Turret/Follow/UsingSpecificTag", false);
        if (vision.hasAnyTarget(cameraIndex)) {
            Logger.recordOutput("Turret/Follow/TargetTagSeen", true);
            Rotation2d tx = vision.getTargetX(cameraIndex);
            Logger.recordOutput("Turret/Follow/TxDeg", tx.getDegrees());
            SmartDashboard.putNumber("Turret/Follow/TxDeg", tx.getDegrees());
            turret.setTargetAngleRadians(turret.getPositionRadians() + tx.getRadians() - followOffsetRad);
        } else {
            Logger.recordOutput("Turret/Follow/TargetTagSeen", false);
            SmartDashboard.putNumber("Turret/Follow/TxDeg", 0.0);
            turret.setTargetAngleRadians(turret.getPositionRadians());
        }
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

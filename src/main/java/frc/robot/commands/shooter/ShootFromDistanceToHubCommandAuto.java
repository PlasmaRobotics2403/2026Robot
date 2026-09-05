package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class ShootFromDistanceToHubCommandAuto extends Command {
    private final Shooter shooter;
    private final IndexerSubsystem indexer;
    private final Drive drive;
    private final TestTurretSubsystem turret;

    public ShootFromDistanceToHubCommandAuto(
            Shooter shooter, IndexerSubsystem indexer, Drive drive, TestTurretSubsystem turret) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.turret = turret;

        // Do not require drive so this command can run at the same time as a PathPlanner path.
        addRequirements(shooter, indexer, turret);
    }

    @Override
    public void initialize() {
        stopFeeders();
        Logger.recordOutput("AutoShoot/CommandActive", true);
    }

    @Override
    public void execute() {
        boolean shotSolutionValid = HubShotController.updateShotSolution(shooter, turret, drive);

        boolean flywheelReady = shotSolutionValid && shooter.isFlywheelAtSetpoint();
        boolean turretUnwinding = turret.isUnwinding();
        boolean feeding = shouldFeed(shotSolutionValid, flywheelReady, turretUnwinding);
        if (feeding) {
            indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
            indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);
        } else {
            stopFeeders();
        }

        Logger.recordOutput("AutoShoot/FlywheelReady", flywheelReady);
        Logger.recordOutput("AutoShoot/TurretUnwinding", turretUnwinding);
        Logger.recordOutput("AutoShoot/Feeding", feeding);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(ShooterConstants.HOOD_TARGET_DEGREES_DASHBOARD_DEFAULT);
        turret.clearUnwinding();
        stopFeeders();
        Logger.recordOutput("AutoShoot/CommandActive", false);
        Logger.recordOutput("AutoShoot/FlywheelReady", false);
        Logger.recordOutput("AutoShoot/Feeding", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void stopFeeders() {
        indexer.stopSpindexer();
        indexer.stopShooterIndexer();
    }

    static boolean shouldFeed(boolean shotSolutionValid, boolean flywheelReady, boolean turretUnwinding) {
        return shotSolutionValid && flywheelReady && !turretUnwinding;
    }
}

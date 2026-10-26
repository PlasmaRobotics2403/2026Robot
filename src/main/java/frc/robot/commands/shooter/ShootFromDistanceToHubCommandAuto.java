package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
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
    private final Timer timer = new Timer();

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
        timer.restart();
        stopFeeders();
        Logger.recordOutput("AutoShoot/CommandActive", true);
        HubShotController.updateShotSolution(shooter, turret, drive);
    }

    @Override
    public void execute() {
        boolean shotSolutionValid = HubShotController.updateShotSolution(shooter, turret, drive);
        boolean feeding = timer.hasElapsed(0.5);
        if (feeding) {
            indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
            indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);
        } else {
            stopFeeders();
        }
        boolean turretUnwinding = turret.isUnwinding();

        Logger.recordOutput("AutoShoot/ShotSolutionValid", shotSolutionValid);
        Logger.recordOutput("AutoShoot/FlywheelReady", shotSolutionValid && shooter.isFlywheelAtSetpoint());
        Logger.recordOutput("AutoShoot/TurretUnwinding", turretUnwinding);
        Logger.recordOutput("AutoShoot/Feeding", feeding);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
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

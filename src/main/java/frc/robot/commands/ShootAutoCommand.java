package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAutoCommand extends Command {
    private final Shooter shooter;
    private final IndexerSubsystem indexer;

    public ShootAutoCommand(Shooter shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        shooter.runShotFromDistance();

        if (shooter.atFlywheelSpeed(
                shooter.evaluateFlywheelRps(shooter.evaluateHoodDegrees()),
                ShooterConstants.FLYWHEEL_SPEED_TOLERANCE_RPS)) {
            indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
            indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);
        } else {
            indexer.stopSpindexer();
            indexer.stopShooterIndexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopSpindexer();
        indexer.stopShooterIndexer();
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

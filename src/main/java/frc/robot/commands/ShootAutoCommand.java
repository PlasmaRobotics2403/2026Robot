package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAutoCommand extends Command {
    private final Shooter shooter;
    private final IndexerSubsystem indexer;
    private Timer timer = new Timer();

    public ShootAutoCommand(Shooter shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        timer.start();
    }

    @Override
    public void execute() {
        shooter.runShot(0.5, 55);
        if (timer.hasElapsed(0.5)) {
            indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
            indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);
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
        return timer.hasElapsed(7);
    }
}

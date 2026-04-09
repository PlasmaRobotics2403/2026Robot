package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class ShootStopCommand extends Command {
    private final Shooter shooter;
    private final IndexerSubsystem indexer;

    public ShootStopCommand(Shooter shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        shooter.runShot(0, 0);
        indexer.setSpindexerDutyCycle(0);
        indexer.setShooterIndexerDutyCycle(0);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        shooter.runShot(0, 0);
        indexer.setSpindexerDutyCycle(0);
        indexer.setShooterIndexerDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

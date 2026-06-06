package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexterDutyCycle extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private double spindexterSpeed;
    private double indexterSpeed;

    public RunIndexterDutyCycle(IndexerSubsystem indexerSubsystem, double spindexterSpeed, double indexterSpeed) {
        this.indexerSubsystem = indexerSubsystem;
        this.spindexterSpeed = spindexterSpeed;
        this.indexterSpeed = indexterSpeed;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        indexerSubsystem.setSpindexerDutyCycle(spindexterSpeed);
        indexerSubsystem.setShooterIndexerDutyCycle(indexterSpeed);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopSpindexer();
        indexerSubsystem.stopShooterIndexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

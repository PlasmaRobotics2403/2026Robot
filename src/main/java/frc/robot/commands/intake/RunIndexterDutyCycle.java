package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import java.util.function.BooleanSupplier;

public class RunIndexterDutyCycle extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private double spindexterSpeed;
    private double indexterSpeed;
    private final BooleanSupplier feedAllowedSupplier;

    public RunIndexterDutyCycle(IndexerSubsystem indexerSubsystem, double spindexterSpeed, double indexterSpeed) {
        this(indexerSubsystem, spindexterSpeed, indexterSpeed, () -> true);
    }

    public RunIndexterDutyCycle(
            IndexerSubsystem indexerSubsystem,
            double spindexterSpeed,
            double indexterSpeed,
            BooleanSupplier feedAllowedSupplier) {
        this.indexerSubsystem = indexerSubsystem;
        this.spindexterSpeed = spindexterSpeed;
        this.indexterSpeed = indexterSpeed;
        this.feedAllowedSupplier = feedAllowedSupplier;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        execute();
    }

    @Override
    public void execute() {
        if (feedAllowedSupplier.getAsBoolean()) {
            indexerSubsystem.setSpindexerDutyCycle(spindexterSpeed);
            indexerSubsystem.setShooterIndexerDutyCycle(indexterSpeed);
        } else {
            indexerSubsystem.stopSpindexer();
            indexerSubsystem.stopShooterIndexer();
        }
    }

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

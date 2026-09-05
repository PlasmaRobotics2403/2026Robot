package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallsCommand extends Command {
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;

    public IntakeBallsCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
        intake.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
        intake.runRollersIn(IntakeConstants.ROLLER_PERCENT);
    }

    @Override
    public void execute() {
        indexer.setSpindexerDutyCycle(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

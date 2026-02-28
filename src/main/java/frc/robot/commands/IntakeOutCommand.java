package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command {
    private IntakeSubsystem intake;

    public IntakeOutCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
    }

    @Override
    public void execute() {
        intake.runRollersIn(IntakeConstants.ROLLER_PERCENT);
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

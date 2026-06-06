package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class OutakeBallsCommand extends Command {
    private IntakeSubsystem intake;
    private final Timer timer = new Timer();

    public OutakeBallsCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.3)) {
            intake.runRollersIn(-IntakeConstants.ROLLER_PERCENT);
        } else {
            intake.stopRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotTargetDegrees(-IntakeConstants.DEPLOY_DEG);
        intake.stopRoller();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

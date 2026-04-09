package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePulseCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private Timer timer;

    public IntakePulseCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        Timer timer = new Timer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intakeSubsystem.setPivotTargetDegrees(IntakeConstants.STOW_DEG);
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(2.5)) {
            timer.reset();
            timer.start();
        }

        if (timer.hasElapsed(0.5)) {
            intakeSubsystem.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
        }

        if (timer.hasElapsed(2.5)) {
            intakeSubsystem.setPivotTargetDegrees(IntakeConstants.STOW_DEG);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

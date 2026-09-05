package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

/** Runs the intake independently so a shooting command can own the indexer. */
public class IntakeOnlyCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeOnlyCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotTargetDegrees(IntakeConstants.DEPLOY_DEG);
        intake.runRollersIn(IntakeConstants.ROLLER_PERCENT);
        Logger.recordOutput("Intake/IntakeOnlyActive", true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
        Logger.recordOutput("Intake/IntakeOnlyActive", false);
    }
}

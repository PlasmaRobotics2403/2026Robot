package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private static final double DEFAULT_DEPLOY_DEG = 95;
  private static final double DEFAULT_STOW_DEG = 0.0;
  private static final double DEFAULT_ROLLER_PERCENT = 0.5;

  private final IntakeSubsystem intake;
  private final double deployDeg;
  private final double stowDeg;
  private final double rollerPercent;

  public IntakeCommand(IntakeSubsystem intake) {
    this(intake, DEFAULT_DEPLOY_DEG, DEFAULT_STOW_DEG, DEFAULT_ROLLER_PERCENT);
  }

  public IntakeCommand(
      IntakeSubsystem intake, double deployDeg, double stowDeg, double rollerPercent) {
    this.intake = intake;
    this.deployDeg = deployDeg;
    this.stowDeg = stowDeg;
    this.rollerPercent = rollerPercent;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setPivotTargetDegrees(deployDeg);
  }

  @Override
  public void execute() {
    intake.setPivotTargetDegrees(deployDeg);
    intake.runRollersIn(rollerPercent);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setPivotTargetDegrees(stowDeg);
    intake.stopRoller();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

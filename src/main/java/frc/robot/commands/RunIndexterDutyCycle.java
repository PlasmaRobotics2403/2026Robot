// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

/** An example command that uses an example subsystem. */
public class RunIndexterDutyCycle extends Command {
  private final IndexerSubsystem indexerSubsystem;
  private double spindexterSpeed;
  private double indexterSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIndexterDutyCycle(
      IndexerSubsystem indexerSubsystem, double spindexterSpeed, double indexterSpeed) {
    this.indexerSubsystem = indexerSubsystem;
    this.spindexterSpeed = spindexterSpeed;
    this.indexterSpeed = indexterSpeed;
    addRequirements(indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSubsystem.setSpindexerDutyCycle(spindexterSpeed);
    indexerSubsystem.setShooterIndexerDutyCycle(indexterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopSpindexer();
    indexerSubsystem.stopShooterIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

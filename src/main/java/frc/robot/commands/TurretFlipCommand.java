// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

/** An example command that uses an example subsystem. */
public class TurretFlipCommand extends Command {
  private final TestTurretSubsystem turret;

  // Target on the opposite side of the wrap range ("option 2")
  private double flippedAngleRad = 0.0;

  // How far away from the hard limit we want to end up after flipping
  private static final double K_UNWIND_MARGIN_RAD = Math.toRadians(15.0);

  // Finish tolerance
  private static final double K_TOLERANCE_RAD = Math.toRadians(7.0);

  public TurretFlipCommand(TestTurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void initialize() {
    double currentAngleRad = turret.getPositionRadians();

    // If we're on the + side, jump to the - side (and vice versa), keeping a margin
    // so we don't sit right on the limit.
    if (currentAngleRad >= 0.0) {
      flippedAngleRad = turret.MIN_ANGLE_RAD + K_UNWIND_MARGIN_RAD;
    } else {
      flippedAngleRad = turret.MAX_ANGLE_RAD - K_UNWIND_MARGIN_RAD;
    }

    turret.setTargetAngleRadians(flippedAngleRad);
  }

  @Override
  public void execute() {
    // Keep target applied in case something else adjusts it.
    turret.setTargetAngleRadians(flippedAngleRad);
    SmartDashboard.putBoolean(
        "TurretAtTarget",
        Math.abs(turret.getPositionRadians() - flippedAngleRad) <= K_TOLERANCE_RAD);
    SmartDashboard.putNumber("FlippedAngle", flippedAngleRad);
    SmartDashboard.putNumber("TurretPosRad", turret.getPositionRadians());
  }

  // Returns true when the command should end.

  @Override
  public void end(boolean interrupted) {
    // Keep holding position at whatever target we were heading toward.
    turret.setTargetAngleRadians(flippedAngleRad);
  }

  @Override
  public boolean isFinished() {
    // Finish immediately; the follow command can continue running while the turret moves.
    return true;
  }
}

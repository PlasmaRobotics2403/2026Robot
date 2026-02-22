// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

/** An example command that uses an example subsystem. */
public class TurretFlipCommand extends Command {
    private final TestTurretSubsystem turret;

    // Must be > TestTurretSubsystem.isAtLimit() band (6 deg) so we don't immediately re-trigger.
    private static final double OPPOSITE_LIMIT_MARGIN_DEG = 25.0;
    private static final double FINISH_TOLERANCE_DEG = 8.0;
    private static final double TIMEOUT_SEC = 2.0;

    private double targetAngleRad = 0.0;
    private double startTimeSec = 0.0;

    public TurretFlipCommand(TestTurretSubsystem turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        startTimeSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        double currentAngleRad = turret.getPositionRadians();
        double marginRad = Math.toRadians(OPPOSITE_LIMIT_MARGIN_DEG);

        // If we're on the + side, unwind to near the - limit. If on the - side, unwind to near the +
        // limit.
        if (currentAngleRad >= 0.0) {
            targetAngleRad = turret.MIN_ANGLE_RAD + marginRad;
        } else {
            targetAngleRad = turret.MAX_ANGLE_RAD - marginRad;
        }

        turret.setTargetAngleRadians(targetAngleRad);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Nothing to do; the subsystem's closed-loop control will drive toward the target.
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Ensure turret stops if the command is canceled (e.g., A released mid-flip).
        turret.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Stay scheduled until we reach the unwind target so the follow command pauses tracking.
        double errRad = Math.abs(turret.getPositionRadians() - targetAngleRad);
        if (errRad <= Math.toRadians(FINISH_TOLERANCE_DEG)) {
            return true;
        }

        // Failsafe: never stay scheduled forever.
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return (now - startTimeSec) >= TIMEOUT_SEC;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

public class ZeroTurretCommand extends Command {

    private TestTurretSubsystem turret;
    boolean targetDirection;

    public ZeroTurretCommand(TestTurretSubsystem turret) {
        this.turret = turret;
        this.targetDirection = false;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        if (Math.toDegrees(turret.getPositionRadians()) > 0) {
            targetDirection = true; // right
        } else {
            targetDirection = false; // left
        }
    }

    @Override
    public void execute() {
        if (targetDirection == true) {
            turret.setDutyCycle(-0.25);
        } else {
            turret.setDutyCycle(0.25);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return turret.getLimitSwitch();
    }
}

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestTurretSubsystem;

public class ZeroTurretCommandRight extends Command {

    private TestTurretSubsystem turret;
    boolean targetDirection;

    public ZeroTurretCommandRight(TestTurretSubsystem turret) {
        this.turret = turret;
        this.targetDirection = false;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        targetDirection = true; // right
    }

    @Override
    public void execute() {
        if (targetDirection == true) {
            turret.setDutyCycle(-0.1);
        } else {
            turret.setDutyCycle(0.1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setDutyCycle(0);
        turret.resetPosition();
    }

    @Override
    public boolean isFinished() {
        return turret.getLimitSwitch();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TestTurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAutoCommand extends Command {
    private final Shooter shooter;
    private final IndexerSubsystem indexer;
    private final Drive drive;
    private final TestTurretSubsystem turret;
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();

    public ShootAutoCommand(
            Shooter shooter,
            IndexerSubsystem indexer,
            Drive drive,
            TestTurretSubsystem turret,
            IntakeSubsystem intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.turret = turret;
        this.intake = intake;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        timer.start();
    }

    @Override
    public void execute() {
        // Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // double hubX, hubY;
        // if (alliance == Alliance.Blue) {
        //     hubX = Constants.blueHubX;
        //     hubY = Constants.blueHubY;
        // } else {
        //     hubX = Constants.redHubX;
        //     hubY = Constants.redHubY;
        // }
        // Translation2d hubField = new Translation2d(hubX, hubY);

        if (drive.getPose().getY() < 4) {
            turret.setTargetAngleRadians(Math.toRadians(-24));
        } else {
            turret.setTargetAngleRadians(Math.toRadians(5));
        }
        indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
        indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);

        shooter.runShot(0.5, 55);
        // if (timer.hasElapsed(0.2)) {
        //     indexer.setSpindexerDutyCycle(ShooterConstants.SPINDEXER_FEED_DUTY);
        //     indexer.setShooterIndexerDutyCycle(ShooterConstants.SHOOTER_KICKER_FEED_DUTY);
        // }

        if (timer.hasElapsed(3)) {
            intake.setPivotTargetDegrees(IntakeConstants.STOW_DEG);
            intake.stopRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopSpindexer();
        indexer.stopShooterIndexer();
        shooter.stopFlywheel();
        shooter.setHoodAngleDegrees(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(7);
    }
}

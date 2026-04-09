package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private static final int SPINDEXER_CAN_ID = 24;
    private static final String SPINDEXER_CAN_BUS = "rio";

    private static final int SHOOTER_INDEXER_CAN_ID = 25;
    private static final String SHOOTER_INDEXER_CAN_BUS = "rio";

    private static final double INDEXER_KP = 0.1;
    private static final double INDEXER_KI = 0.0;
    private static final double INDEXER_KD = 0.0;
    private static final double INDEXER_KS = 0.25;
    private static final double INDEXER_KV = 0.12;
    private static final double INDEXER_STATOR_CURRENT_LIMIT_AMPS = 120.0;
    private static final double INDEXER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    private static final double INDEXER_SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 40.0;
    private static final double INDEXER_SUPPLY_CURRENT_LOWER_TIME_SEC = 1.0;

    private final TalonFX spindexerMotor = new TalonFX(SPINDEXER_CAN_ID, new CANBus(SPINDEXER_CAN_BUS));
    private final TalonFX shooterIndexerMotor =
            new TalonFX(SHOOTER_INDEXER_CAN_ID, new CANBus(SHOOTER_INDEXER_CAN_BUS));

    private final DutyCycleOut spindexerDutyCycleRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut shooterIndexerDutyCycleRequest = new DutyCycleOut(0.0);

    private final VelocityTorqueCurrentFOC spindexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC shooterIndexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    public IndexerSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = INDEXER_KP;
        config.Slot0.kI = INDEXER_KI;
        config.Slot0.kD = INDEXER_KD;
        config.Slot0.kS = INDEXER_KS;
        config.Slot0.kV = INDEXER_KV;
        config.CurrentLimits.StatorCurrentLimit = INDEXER_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = INDEXER_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerLimit = INDEXER_SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerTime = INDEXER_SUPPLY_CURRENT_LOWER_TIME_SEC;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        spindexerMotor.getConfigurator().apply(config);
        shooterIndexerMotor.getConfigurator().apply(config);
    }

    public void index(double speed) {
        setSpindexerDutyCycle(speed);
        setShooterIndexerDutyCycle(speed);
    }

    public void setSpindexerDutyCycle(double dutyCycle) {
        double clamped = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        spindexerMotor.setControl(spindexerDutyCycleRequest.withOutput(clamped));
    }

    public void setSpindexerVelocityMotorRps(double motorRps) {
        spindexerMotor.setControl(spindexerVelocityRequest.withVelocity(RotationsPerSecond.of(motorRps)));
    }

    public void setShooterIndexerDutyCycle(double dutyCycle) {
        double clamped = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        shooterIndexerMotor.setControl(shooterIndexerDutyCycleRequest.withOutput(clamped));
    }

    public void setShooterIndexerVelocityMotorRps(double motorRps) {
        shooterIndexerMotor.setControl(shooterIndexerVelocityRequest.withVelocity(RotationsPerSecond.of(motorRps)));
    }

    public void stopSpindexer() {
        setSpindexerDutyCycle(0);
    }

    public void stopShooterIndexer() {
        shooterIndexerMotor.stopMotor();
    }
}

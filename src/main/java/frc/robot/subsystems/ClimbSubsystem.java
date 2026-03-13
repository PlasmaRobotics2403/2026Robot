package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private double targetPositionRotations = ClimbConstants.HOME_POSITION_ROTATIONS;

    public ClimbSubsystem() {
        climbMotor = new TalonFX(ClimbConstants.MOTOR_CAN_ID, new CANBus(ClimbConstants.CANBUS_NAME));

        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = ClimbConstants.MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        climbConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT_AMPS;
        climbConfig.CurrentLimits.SupplyCurrentLimitEnable = ClimbConstants.SUPPLY_CURRENT_LIMIT_ENABLED;
        climbConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT_AMPS;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = ClimbConstants.STATOR_CURRENT_LIMIT_ENABLED;

        climbConfig.Slot0.kP = ClimbConstants.POSITION_KP;
        climbConfig.Slot0.kI = ClimbConstants.POSITION_KI;
        climbConfig.Slot0.kD = ClimbConstants.POSITION_KD;
        climbConfig.Slot0.kS = ClimbConstants.POSITION_KS;
        climbConfig.Slot0.kV = ClimbConstants.POSITION_KV;
        climbConfig.Slot0.kA = ClimbConstants.POSITION_KA;

        climbMotor.getConfigurator().apply(climbConfig);
        climbMotor.setPosition(ClimbConstants.HOME_POSITION_ROTATIONS);
    }

    public double getClimbPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    public void setTargetPositionRotations(double targetPositionRotations) {
        this.targetPositionRotations = targetPositionRotations;
        climbMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    public double getTargetPositionRotations() {
        return targetPositionRotations;
    }

    public void setDutyCycle(double dutyCycle) {
        double clampedDutyCycle =
                MathUtil.clamp(dutyCycle, -ClimbConstants.MAX_DUTY_CYCLE, ClimbConstants.MAX_DUTY_CYCLE);
        climbMotor.setControl(dutyCycleRequest.withOutput(clampedDutyCycle));
    }

    public boolean atTargetPosition() {
        return Math.abs(getClimbPosition() - targetPositionRotations) <= ClimbConstants.POSITION_TOLERANCE_ROTATIONS;
    }

    public void resetPosition(double positionRotations) {
        targetPositionRotations = positionRotations;
        climbMotor.setPosition(positionRotations);
    }

    public void stop() {
        climbMotor.stopMotor();
    }
}

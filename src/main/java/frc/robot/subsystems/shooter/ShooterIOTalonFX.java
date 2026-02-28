package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX flywheelMotor;
    private final TalonFX hoodMotor;

    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
    private final DutyCycleOut flywheelDutyRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut hoodDutyRequest = new DutyCycleOut(0.0);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelSupplyCurrent;
    private final StatusSignal<Current> flywheelStatorCurrent;
    private final StatusSignal<Temperature> flywheelTemp;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodSupplyCurrent;
    private final StatusSignal<Temperature> hoodTemp;

    private final Debouncer flywheelConnectedDebounce = new Debouncer(0.5);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5);

    public ShooterIOTalonFX() {
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_CAN_ID, new CANBus(ShooterConstants.CANBUS_NAME));
        hoodMotor = new TalonFX(ShooterConstants.HOOD_CAN_ID, new CANBus(ShooterConstants.CANBUS_NAME));

        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        flywheelConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.FLYWHEEL_KP)
                .withKI(ShooterConstants.FLYWHEEL_KI)
                .withKD(ShooterConstants.FLYWHEEL_KD)
                .withKS(ShooterConstants.FLYWHEEL_KS)
                .withKV(ShooterConstants.FLYWHEEL_KV)
                .withKA(ShooterConstants.FLYWHEEL_KA);

        tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = ShooterConstants.HOOD_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_CURRENT_LIMIT;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.HOOD_KP)
                .withKI(ShooterConstants.HOOD_KI)
                .withKD(ShooterConstants.HOOD_KD)
                .withKS(ShooterConstants.HOOD_KS)
                .withKV(ShooterConstants.HOOD_KV)
                .withKA(ShooterConstants.HOOD_KA);

        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));

        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();
        flywheelStatorCurrent = flywheelMotor.getStatorCurrent();
        flywheelTemp = flywheelMotor.getDeviceTemp();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
        hoodTemp = hoodMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                flywheelVelocity,
                flywheelAppliedVolts,
                flywheelSupplyCurrent,
                flywheelStatorCurrent,
                flywheelTemp,
                hoodPosition,
                hoodVelocity,
                hoodAppliedVolts,
                hoodSupplyCurrent,
                hoodTemp);
        ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, hoodMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var flywheelStatus = BaseStatusSignal.refreshAll(
                flywheelVelocity, flywheelAppliedVolts, flywheelSupplyCurrent, flywheelStatorCurrent, flywheelTemp);
        var hoodStatus =
                BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodSupplyCurrent, hoodTemp);

        inputs.flywheelConnected = flywheelConnectedDebounce.calculate(flywheelStatus.isOK());
        inputs.flywheelVelocityRps = flywheelVelocity.getValueAsDouble();
        inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValueAsDouble();
        inputs.flywheelSupplyCurrentAmps = flywheelSupplyCurrent.getValueAsDouble();
        inputs.flywheelStatorCurrentAmps = flywheelStatorCurrent.getValueAsDouble();
        inputs.flywheelTempCelsius = flywheelTemp.getValueAsDouble();
        inputs.flywheelClosedLoopErrorRps = flywheelMotor.getClosedLoopError().getValueAsDouble();

        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
        inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();
        inputs.hoodVelocityRps = hoodVelocity.getValueAsDouble();
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
        inputs.hoodCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = hoodTemp.getValueAsDouble();
        inputs.hoodClosedLoopErrorRotations = hoodMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public void setFlywheelVelocityRps(double rps) {
        flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(rps));
    }

    @Override
    public void setFlywheelDutyCycle(double output) {
        flywheelMotor.setControl(flywheelDutyRequest.withOutput(output));
    }

    @Override
    public void setHoodDutyCycle(double output) {
        hoodMotor.setControl(hoodDutyRequest.withOutput(output));
    }

    @Override
    public void setHoodPositionRotations(double rotations) {
        hoodMotor.setControl(hoodPositionRequest.withPosition(rotations));
    }

    @Override
    public void stopFlywheel() {
        flywheelMotor.setControl(neutralRequest);
    }

    @Override
    public void stopHood() {
        hoodMotor.setControl(neutralRequest);
    }
}

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

    private final TalonFX flywheelLeaderMotor;
    private final TalonFX flywheelFollowerMotor;
    private final TalonFX hoodMotor;
    private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
    private final DutyCycleOut flywheelDutyRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut hoodDutyRequest = new DutyCycleOut(0.0);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<AngularVelocity> flywheelLeaderVelocity;
    private final StatusSignal<Voltage> flywheelLeaderAppliedVolts;
    private final StatusSignal<Current> flywheelLeaderSupplyCurrent;
    private final StatusSignal<Current> flywheelLeaderStatorCurrent;
    private final StatusSignal<Temperature> flywheelLeaderTemp;
    private final StatusSignal<AngularVelocity> flywheelFollowerVelocity;
    private final StatusSignal<Voltage> flywheelFollowerAppliedVolts;
    private final StatusSignal<Current> flywheelFollowerSupplyCurrent;
    private final StatusSignal<Current> flywheelFollowerStatorCurrent;
    private final StatusSignal<Temperature> flywheelFollowerTemp;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodSupplyCurrent;
    private final StatusSignal<Temperature> hoodTemp;

    private final Debouncer flywheelLeaderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer flywheelFollowerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5);

    public ShooterIOTalonFX() {
        flywheelLeaderMotor =
                new TalonFX(ShooterConstants.FLYWHEEL_LEADER_CAN_ID, new CANBus(ShooterConstants.CANBUS_NAME));
        flywheelFollowerMotor =
                new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_CAN_ID, new CANBus(ShooterConstants.CANBUS_NAME));
        hoodMotor = new TalonFX(ShooterConstants.HOOD_CAN_ID, new CANBus(ShooterConstants.CANBUS_NAME));

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_LEADER_INVERTED
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

        tryUntilOk(5, () -> flywheelLeaderMotor.getConfigurator().apply(flywheelConfig, 0.25));
        flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_FOLLOWER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelConfig, 0.25));

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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

        flywheelLeaderVelocity = flywheelLeaderMotor.getVelocity();
        flywheelLeaderAppliedVolts = flywheelLeaderMotor.getMotorVoltage();
        flywheelLeaderSupplyCurrent = flywheelLeaderMotor.getSupplyCurrent();
        flywheelLeaderStatorCurrent = flywheelLeaderMotor.getStatorCurrent();
        flywheelLeaderTemp = flywheelLeaderMotor.getDeviceTemp();
        flywheelFollowerVelocity = flywheelFollowerMotor.getVelocity();
        flywheelFollowerAppliedVolts = flywheelFollowerMotor.getMotorVoltage();
        flywheelFollowerSupplyCurrent = flywheelFollowerMotor.getSupplyCurrent();
        flywheelFollowerStatorCurrent = flywheelFollowerMotor.getStatorCurrent();
        flywheelFollowerTemp = flywheelFollowerMotor.getDeviceTemp();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
        hoodTemp = hoodMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                flywheelLeaderVelocity,
                flywheelLeaderAppliedVolts,
                flywheelLeaderSupplyCurrent,
                flywheelLeaderStatorCurrent,
                flywheelLeaderTemp,
                flywheelFollowerVelocity,
                flywheelFollowerAppliedVolts,
                flywheelFollowerSupplyCurrent,
                flywheelFollowerStatorCurrent,
                flywheelFollowerTemp,
                hoodPosition,
                hoodVelocity,
                hoodAppliedVolts,
                hoodSupplyCurrent,
                hoodTemp);
        ParentDevice.optimizeBusUtilizationForAll(flywheelLeaderMotor, flywheelFollowerMotor, hoodMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var flywheelLeaderStatus = BaseStatusSignal.refreshAll(
                flywheelLeaderVelocity,
                flywheelLeaderAppliedVolts,
                flywheelLeaderSupplyCurrent,
                flywheelLeaderStatorCurrent,
                flywheelLeaderTemp);
        var flywheelFollowerStatus = BaseStatusSignal.refreshAll(
                flywheelFollowerVelocity,
                flywheelFollowerAppliedVolts,
                flywheelFollowerSupplyCurrent,
                flywheelFollowerStatorCurrent,
                flywheelFollowerTemp);
        var hoodStatus =
                BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodSupplyCurrent, hoodTemp);

        boolean leaderConnected = flywheelLeaderConnectedDebounce.calculate(flywheelLeaderStatus.isOK());
        boolean followerConnected = flywheelFollowerConnectedDebounce.calculate(flywheelFollowerStatus.isOK());
        inputs.flywheelConnected = leaderConnected && followerConnected;
        inputs.flywheelLeaderVelocityRps = flywheelLeaderVelocity.getValueAsDouble();
        inputs.flywheelFollowerVelocityRps = flywheelFollowerVelocity.getValueAsDouble();
        inputs.flywheelVelocityRps = (inputs.flywheelLeaderVelocityRps + inputs.flywheelFollowerVelocityRps) / 2.0;
        inputs.flywheelAppliedVolts =
                (flywheelLeaderAppliedVolts.getValueAsDouble() + flywheelFollowerAppliedVolts.getValueAsDouble()) / 2.0;
        inputs.flywheelSupplyCurrentAmps =
                flywheelLeaderSupplyCurrent.getValueAsDouble() + flywheelFollowerSupplyCurrent.getValueAsDouble();
        inputs.flywheelStatorCurrentAmps =
                flywheelLeaderStatorCurrent.getValueAsDouble() + flywheelFollowerStatorCurrent.getValueAsDouble();
        inputs.flywheelTempCelsius =
                Math.max(flywheelLeaderTemp.getValueAsDouble(), flywheelFollowerTemp.getValueAsDouble());
        inputs.flywheelClosedLoopErrorRps =
                (flywheelLeaderMotor.getClosedLoopError().getValueAsDouble()
                                + flywheelFollowerMotor.getClosedLoopError().getValueAsDouble())
                        / 2.0;

        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
        inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();
        inputs.hoodVelocityRps = hoodVelocity.getValueAsDouble();
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
        inputs.hoodCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = hoodTemp.getValueAsDouble();
        inputs.hoodClosedLoopErrorRotations = hoodMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public void setFlywheelPid(double kP, double kI, double kD) {
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kI = kI;
        flywheelConfig.Slot0.kD = kD;
        flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_LEADER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> flywheelLeaderMotor.getConfigurator().apply(flywheelConfig, 0.25));
        flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_FOLLOWER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelConfig, 0.25));
    }

    @Override
    public void setHoodPid(double kP, double kI, double kD) {
        hoodConfig.Slot0.kP = kP;
        hoodConfig.Slot0.kI = kI;
        hoodConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    }

    @Override
    public void setFlywheelVelocityRps(double rps) {
        flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(rps));
        flywheelFollowerMotor.setControl(flywheelVelocityRequest.withVelocity(rps));
    }

    @Override
    public void setFlywheelDutyCycle(double output) {
        flywheelLeaderMotor.setControl(flywheelDutyRequest.withOutput(output));
        flywheelFollowerMotor.setControl(flywheelDutyRequest.withOutput(output));
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
        flywheelLeaderMotor.setControl(neutralRequest);
        flywheelFollowerMotor.setControl(neutralRequest);
    }

    @Override
    public void stopHood() {
        hoodMotor.setControl(neutralRequest);
    }
}

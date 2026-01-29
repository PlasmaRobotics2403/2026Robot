package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class TestTurretSubsystem extends RBSISubsystem {
	public static final int TALON_ID = 20;

	public static final String CAN_BUS = "rio";
	public static final double GEAR_RATIO = 1.0;
	private final TalonFX motor = new TalonFX(TALON_ID, CAN_BUS);
	private final TalonFXConfiguration config = new TalonFXConfiguration();

	private final StatusSignal<Angle> position = motor.getPosition();
	private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
	private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
	private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();

	private final VoltageOut voltageRequest = new VoltageOut(0.0);

	public TestTurretSubsystem() {
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;

		motor.getConfigurator().apply(config);

		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0, position, velocity, appliedVolts, supplyCurrent);
		motor.optimizeBusUtilization();

		Logger.recordOutput("Turret/Mode", Constants.getMode().toString());
	}

	@Override
	public void periodic() {
		BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent);
		Logger.recordOutput("Turret/PositionRot", getPositionRotations());
		Logger.recordOutput("Turret/VelocityRotPerSec", getVelocityRotationsPerSecond());
		Logger.recordOutput("Turret/AppliedVolts", appliedVolts.getValueAsDouble());
		Logger.recordOutput("Turret/SupplyCurrentAmps", supplyCurrent.getValueAsDouble());
	}

	public void setVoltage(double volts) {
		motor.setControl(voltageRequest.withOutput(volts));
	}

	public void stop() {
		motor.stopMotor();
	}

	/** Returns turret output rotations (post-gearbox). */
	@AutoLogOutput(key = "Turret/PositionRot")
	public double getPositionRotations() {
			return position.getValueAsDouble() / GEAR_RATIO;
	}

	/** Returns turret output radians (post-gearbox). */
	public double getPositionRadians() {
		return Units.rotationsToRadians(getPositionRotations());
	}

	/** Returns turret output rotations/sec (post-gearbox). */
	@AutoLogOutput(key = "Turret/VelocityRotPerSec")
	public double getVelocityRotationsPerSecond() {
			return velocity.getValueAsDouble() / GEAR_RATIO;
	}

	/** Returns turret output rad/s (post-gearbox). */
	public double getVelocityRadiansPerSecond() {
		return Units.rotationsToRadians(getVelocityRotationsPerSecond());
	}
}

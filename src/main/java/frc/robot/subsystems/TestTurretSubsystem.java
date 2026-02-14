package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.DashboardThrottle;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TestTurretSubsystem extends RBSISubsystem {
  public static final int TALON_ID = 20;

  public static final String CAN_BUS = "rio";
  public static final double GEAR_RATIO = 18.5;

  private static final String DASHBOARD_PID_PREFIX = "Turret PID/";

  private static final double POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.0);

  // Soft limits (post-gearbox output angle). Prevents wrapping past the allowed range.
  public final double MIN_ANGLE_RAD = Units.degreesToRadians(-180);
  public final double MAX_ANGLE_RAD = Units.degreesToRadians(180);

  private double targetAngleRad = 0.0;
  private boolean controlEnabled = false;
  private double lastDashboardP = TurretConstants.kP;
  private double lastDashboardI = TurretConstants.kI;
  private double lastDashboardD = TurretConstants.kD;
  private final TalonFX motor = new TalonFX(TALON_ID, CAN_BUS);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  public TestTurretSubsystem() {
    motor.setPosition(0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kI = TurretConstants.kI;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kS = TurretConstants.kS;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kA = TurretConstants.kA;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kCruiseVelocityRps;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.kAccelerationRpsPerSec;
    config.MotionMagic.MotionMagicJerk = TurretConstants.kJerkRpsPerSec2;

    motor.getConfigurator().apply(config);
    motor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent);
    motor.optimizeBusUtilization();

    // Publish tunable PID values to SmartDashboard (editable on the fly)
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kP", TurretConstants.kP);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kI", TurretConstants.kI);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kD", TurretConstants.kD);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kV", TurretConstants.kV);

    Logger.recordOutput("Turret/Mode", Constants.getMode().toString());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent);

    // Live PID tuning from SmartDashboard
    double dashP = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kP", TurretConstants.kP);
    double dashI = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kI", TurretConstants.kI);
    double dashD = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kD", TurretConstants.kD);
    double dashV = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kV", TurretConstants.kV);

    if (dashP != lastDashboardP || dashI != lastDashboardI || dashD != lastDashboardD) {
      config.Slot0.kP = dashP;
      config.Slot0.kI = dashI;
      config.Slot0.kD = dashD;
      motor.getConfigurator().apply(config);

      lastDashboardP = dashP;
      lastDashboardI = dashI;
      lastDashboardD = dashD;
    }

    Logger.recordOutput("Turret/PID/kP", lastDashboardP);
    Logger.recordOutput("Turret/PID/kI", lastDashboardI);
    Logger.recordOutput("Turret/PID/kD", lastDashboardD);
    Logger.recordOutput("Turret/PID/kV", dashV);

    Logger.recordOutput("Turret/velocityRps", getVelocityRotationsPerSecond());
    if (DashboardThrottle.shouldPublish("Turret/Dashboard", 0.1)) {
      SmartDashboard.putBoolean("Turret/AtLimit", isAtLimit());
      SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kP", lastDashboardP);
      SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kI", lastDashboardI);
      SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kD", lastDashboardD);
      SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kV", dashV);
      SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "PID Output", motor.get());
    }
    Logger.recordOutput("Turret/PositionRot", getPositionRotations());
    Logger.recordOutput("Turret/VelocityRotPerSec", getVelocityRotationsPerSecond());
    Logger.recordOutput("Turret/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput("Turret/SupplyCurrentAmps", supplyCurrent.getValueAsDouble());

    // Human-friendly angle logging (post-gearbox)
    double angleDeg = Units.radiansToDegrees(getPositionRadians());
    Logger.recordOutput("Turret/AngleDeg", angleDeg);
    if (DashboardThrottle.shouldPublish("Turret/Angle", 0.1)) {
      SmartDashboard.putNumber("Turret/AngleDeg", angleDeg);
    }

    Logger.recordOutput("Turret/ClosedLoopEnabled", controlEnabled);
    Logger.recordOutput("Turret/TargetAngleRad", targetAngleRad);
    Logger.recordOutput("Turret/TargetAngleDeg", Units.radiansToDegrees(targetAngleRad));
    Logger.recordOutput("Turret/AngleErrorRad", targetAngleRad - getPositionRadians());

    if (DashboardThrottle.shouldPublish("Turret/Target", 0.1)) {
      SmartDashboard.putNumber("Turret/TargetAngleDeg", Units.radiansToDegrees(targetAngleRad));
      SmartDashboard.putBoolean("Turret/ClosedLoopEnabled", controlEnabled);
    }
  }

  /** Sets the turret target angle (post-gearbox), and enables holding that position. */
  public void setTargetAngle(Rotation2d angle) {
    setTargetAngleRadians(angle.getRadians());
  }

  /** Sets the turret target angle in radians (post-gearbox), and enables holding that position. */
  public void setTargetAngleRadians(double angleRad) {
    targetAngleRad = MathUtil.clamp(angleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
    controlEnabled = true;

    double rotorRotations = Units.radiansToRotations(targetAngleRad) * GEAR_RATIO;
    motor.setControl(motionMagicRequest.withPosition(rotorRotations));
  }

  /** Returns the turret target angle in radians (post-gearbox). */
  public double getTargetAngleRadians() {
    return targetAngleRad;
  }

  /** Returns true when the turret is within the configured tolerance of its target. */
  public boolean atTarget() {
    return Math.abs(targetAngleRad - getPositionRadians()) <= POSITION_TOLERANCE_RAD;
  }

  /** Disables closed-loop holding and commands zero output. */
  public void disableClosedLoop() {
    controlEnabled = false;
    stop();
  }

  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  public void stop() {
    controlEnabled = false;
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

  public boolean isAtLimit() {
    double posRad = getPositionRadians();
    return posRad <= MIN_ANGLE_RAD + Math.toRadians(6.0)
        || posRad >= MAX_ANGLE_RAD - Math.toRadians(6.0);
  }
}

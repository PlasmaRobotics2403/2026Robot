package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TestTurretSubsystem extends RBSISubsystem {
  public static final int TALON_ID = 20;

  public static final String CAN_BUS = "rio";
  public static final double GEAR_RATIO = 18.5;

  private static final String DASHBOARD_PID_PREFIX = "Turret PID/";

  // Default PID values (overridden live by SmartDashboard if edited)
  private static final double kP = 3.0;
  private static final double kI = 0.0;
  private static final double kD = 0.03;

  private static final double MAX_CONTROL_VOLTS = 6.0;
  private static final double POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.0);

  // Soft limits (post-gearbox output angle). Prevents wrapping past the allowed range.
  public final double MIN_ANGLE_RAD = Units.degreesToRadians(-180);
  public final double MAX_ANGLE_RAD = Units.degreesToRadians(180);

  private final PIDController angleController = new PIDController(kP, kI, kD);
  private double lastDashboardP = kP;
  private double lastDashboardI = kI;
  private double lastDashboardD = kD;
  private double targetAngleRad = 0.0;
  private boolean holdPositionEnabled = false;
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

    // Reset turret encoder position on boot so angle starts at zero each power cycle.
    // (This assumes the turret is mechanically in its "zero" reference position at startup.)
    motor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent);
    motor.optimizeBusUtilization();

    angleController.setTolerance(POSITION_TOLERANCE_RAD);

    // Publish tunable PID values to SmartDashboard (editable on the fly)
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kP", kP);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kI", kI);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "kD", kD);

    Logger.recordOutput("Turret/Mode", Constants.getMode().toString());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent);

    // Live PID tuning from SmartDashboard
    double dashP = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kP", kP);
    double dashI = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kI", kI);
    double dashD = SmartDashboard.getNumber(DASHBOARD_PID_PREFIX + "kD", kD);

    if (dashP != lastDashboardP || dashI != lastDashboardI || dashD != lastDashboardD) {
      angleController.setPID(dashP, dashI, dashD);
      // Reset to avoid sudden jumps from prior error history while gains change.
      angleController.reset();
      lastDashboardP = dashP;
      lastDashboardI = dashI;
      lastDashboardD = dashD;
    }

    Logger.recordOutput("Turret/PID/kP", lastDashboardP);
    Logger.recordOutput("Turret/PID/kI", lastDashboardI);
    Logger.recordOutput("Turret/PID/kD", lastDashboardD);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kP", lastDashboardP);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kI", lastDashboardI);
    SmartDashboard.putNumber(DASHBOARD_PID_PREFIX + "Active kD", lastDashboardD);
    SmartDashboard.putBoolean("Turret/AtLimit", isAtLimit());
    Logger.recordOutput("Turret/PositionRot", getPositionRotations());
    Logger.recordOutput("Turret/VelocityRotPerSec", getVelocityRotationsPerSecond());
    Logger.recordOutput("Turret/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput("Turret/SupplyCurrentAmps", supplyCurrent.getValueAsDouble());

    // Human-friendly angle logging (post-gearbox)
    double angleDeg = Units.radiansToDegrees(getPositionRadians());
    Logger.recordOutput("Turret/AngleDeg", angleDeg);
    SmartDashboard.putNumber("Turret/AngleDeg", angleDeg);

    Logger.recordOutput("Turret/ClosedLoopEnabled", holdPositionEnabled);
    Logger.recordOutput("Turret/TargetAngleRad", targetAngleRad);
    Logger.recordOutput("Turret/AngleErrorRad", targetAngleRad - getPositionRadians());

    SmartDashboard.putNumber("Turret/TargetAngleDeg", Units.radiansToDegrees(targetAngleRad));
    SmartDashboard.putBoolean("Turret/ClosedLoopEnabled", holdPositionEnabled);

    if (holdPositionEnabled) {
      // Clamp target to soft limits.
      targetAngleRad = MathUtil.clamp(targetAngleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);

      // Hard stop: if we are beyond the soft limit, only allow motion back toward the range.
      double posRad = getPositionRadians();
      double outVolts = angleController.calculate(posRad, targetAngleRad);
      outVolts = MathUtil.clamp(outVolts, -MAX_CONTROL_VOLTS, MAX_CONTROL_VOLTS);

      if (posRad >= MAX_ANGLE_RAD) {
        // Only allow motion that decreases angle.
        outVolts = Math.min(outVolts, 0.0);
      } else if (posRad <= MIN_ANGLE_RAD) {
        // Only allow motion that increases angle.
        outVolts = Math.max(outVolts, 0.0);
      }

      setVoltage(outVolts);
    }
  }

  /** Sets the turret target angle (post-gearbox), and enables holding that position. */
  public void setTargetAngle(Rotation2d angle) {
    targetAngleRad = MathUtil.clamp(angle.getRadians(), MIN_ANGLE_RAD, MAX_ANGLE_RAD);
    // Reset controller so we don't inherit integrator/derivative history.
    angleController.reset();
    holdPositionEnabled = true;
  }

  /** Sets the turret target angle in radians (post-gearbox), and enables holding that position. */
  public void setTargetAngleRadians(double angleRad) {
    setTargetAngle(Rotation2d.fromRadians(angleRad));
  }

  /** Returns the turret target angle in radians (post-gearbox). */
  public double getTargetAngleRadians() {
    return targetAngleRad;
  }

  /** Returns true when the turret is within the configured tolerance of its target. */
  public boolean atTarget() {
    return angleController.atSetpoint();
  }

  /** Disables closed-loop holding and commands zero output. */
  public void disableClosedLoop() {
    holdPositionEnabled = false;
    angleController.reset();
    stop();
  }

  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  public void stop() {
    holdPositionEnabled = false;
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

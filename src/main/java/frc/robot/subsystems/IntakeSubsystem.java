package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotDevices;
import frc.robot.util.DashboardThrottle;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends RBSISubsystem {

  private static final double PIVOT_GEAR_RATIO = 15.3;
  private static final double ROLLER_GEAR_RATIO = 1.0;

  private static final double MAX_PIVOT_VOLTS = 6.0;
  private static final double MAX_ROLLER_VOLTS = 12.0;

  // Start slow for testing.
  private static final double PIVOT_MAX_PID_VOLTS = 2.0;

  private final TalonFX pivotMotor =
      new TalonFX(
          RobotDevices.IntakeConstants.INTAKE_PIVOT.getDeviceNumber(),
          RobotDevices.IntakeConstants.INTAKE_PIVOT.getBus());
  private final TalonFX rollerMotor =
      new TalonFX(
          RobotDevices.IntakeConstants.INTAKE_ROLLER.getDeviceNumber(),
          RobotDevices.IntakeConstants.INTAKE_ROLLER.getBus());

  private final int[] powerPorts =
      new int[] {
        RobotDevices.IntakeConstants.INTAKE_PIVOT.getPowerPort(),
        RobotDevices.IntakeConstants.INTAKE_ROLLER.getPowerPort()
      };

  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

  private final StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> pivotSupplyCurrent = pivotMotor.getSupplyCurrent();

  private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = rollerMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerSupplyCurrent = rollerMotor.getSupplyCurrent();

  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);

  // Last commanded roller output for debugging
  private double rollerCommandVolts = 0.0;
  private double rollerCommandPercent = 0.0;

  // -------------------- Pivot position PID --------------------
  // Units: radians (mechanism/output, NOT motor rotations)
  private final PIDController pivotPid = new PIDController(3.0, 0.0, 0.0);
  private boolean pivotClosedLoopEnabled = false;
  private double pivotTargetRad = 0.0;

  public IntakeSubsystem() {
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotMotor.getConfigurator().apply(pivotConfig);

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(rollerConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrent);
    pivotMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();

    // PID defaults (safe and gentle)
    pivotPid.setTolerance(Units.degreesToRadians(2.0));
    pivotPid.reset();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrent);

    Logger.recordOutput("Intake/Pivot/PositionRad", getPivotPositionRadians());
    Logger.recordOutput("Intake/Pivot/VelocityRadPerSec", getPivotVelocityRadPerSec());
    Logger.recordOutput("Intake/Pivot/AppliedVolts", pivotAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Intake/Pivot/SupplyCurrentAmps", pivotSupplyCurrent.getValueAsDouble());

    if (DashboardThrottle.shouldPublish("Intake/Dashboard", 0.1)) {
      // SmartDashboard "Intake" tab key group
      SmartDashboard.putNumber(
          "Intake/Pivot/AngleDeg", Units.radiansToDegrees(getPivotPositionRadians()));
    }

    Logger.recordOutput("Intake/Roller/VelocityRadPerSec", getRollerVelocityRadPerSec());
    Logger.recordOutput("Intake/Roller/AppliedVolts", rollerAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Intake/Roller/SupplyCurrentAmps", rollerSupplyCurrent.getValueAsDouble());

    if (DashboardThrottle.shouldPublish("Intake/Dashboard", 0.1)) {
      // SmartDashboard "Intake" tab key group
      SmartDashboard.putNumber(
          "Intake/Roller/SpeedRPM", (getRollerVelocityRadPerSec() * 60.0) / (2.0 * Math.PI));
    }

    Logger.recordOutput("Intake/Roller/CommandVolts", rollerCommandVolts);
    Logger.recordOutput("Intake/Roller/CommandPercent", rollerCommandPercent);
    if (DashboardThrottle.shouldPublish("Intake/Commands", 0.1)) {
      SmartDashboard.putNumber("Intake/Roller/CommandVolts", rollerCommandVolts);
      SmartDashboard.putNumber("Intake/Roller/CommandPercent", rollerCommandPercent);
    }

    // Pivot closed-loop control (optional)
    if (pivotClosedLoopEnabled) {
      double measurementRad = getPivotPositionRadians();
      double outputVolts = pivotPid.calculate(measurementRad, pivotTargetRad);
      double errorRad = pivotTargetRad - measurementRad;
      outputVolts = MathUtil.clamp(outputVolts, -PIVOT_MAX_PID_VOLTS, PIVOT_MAX_PID_VOLTS);
      setPivotVoltage(outputVolts);

      Logger.recordOutput("Intake/Pivot/PID/Enabled", true);
      Logger.recordOutput("Intake/Pivot/PID/TargetRad", pivotTargetRad);
      Logger.recordOutput("Intake/Pivot/PID/ErrorRad", errorRad);
      Logger.recordOutput("Intake/Pivot/PID/OutputVolts", outputVolts);

      if (DashboardThrottle.shouldPublish("Intake/PivotPID", 0.1)) {
        SmartDashboard.putBoolean("Intake/Pivot/PID/Enabled", true);
        SmartDashboard.putNumber(
            "Intake/Pivot/PID/TargetDeg", Units.radiansToDegrees(pivotTargetRad));
        SmartDashboard.putNumber("Intake/Pivot/PID/ErrorDeg", Units.radiansToDegrees(errorRad));
        SmartDashboard.putNumber("Intake/Pivot/PID/OutputVolts", outputVolts);
        SmartDashboard.putBoolean("Intake/Pivot/PID/AtTarget", isPivotAtTarget());
      }
    } else {
      Logger.recordOutput("Intake/Pivot/PID/Enabled", false);
      if (DashboardThrottle.shouldPublish("Intake/PivotPID", 0.2)) {
        SmartDashboard.putBoolean("Intake/Pivot/PID/Enabled", false);
      }
    }
  }

  public void setPivotVoltage(double volts) {
    double clamped = MathUtil.clamp(volts, -MAX_PIVOT_VOLTS, MAX_PIVOT_VOLTS);
    pivotMotor.setControl(pivotVoltageRequest.withOutput(clamped));
  }

  /** Enable closed-loop pivot control and immediately hold the current position. */
  public void enablePivotClosedLoopHold() {
    pivotTargetRad = getPivotPositionRadians();
    pivotPid.reset();
    pivotClosedLoopEnabled = true;
  }

  /** Disable closed-loop pivot control (does not automatically stop the motor). */
  public void disablePivotClosedLoop() {
    pivotClosedLoopEnabled = false;
    pivotPid.reset();
  }

  /** Set the desired pivot angle in radians and enable closed-loop control. */
  public void setPivotTargetRadians(double targetRad) {
    pivotTargetRad = targetRad;
    pivotClosedLoopEnabled = true;
  }

  /** Convenience wrapper: set desired pivot angle in degrees and enable closed-loop control. */
  public void setPivotTargetDegrees(double targetDeg) {
    setPivotTargetRadians(Units.degreesToRadians(targetDeg));
  }

  public double getPivotTargetRadians() {
    return pivotTargetRad;
  }

  public boolean isPivotClosedLoopEnabled() {
    return pivotClosedLoopEnabled;
  }

  public boolean isPivotAtTarget() {
    return pivotPid.atSetpoint();
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  @AutoLogOutput(key = "Intake/Pivot/PositionRad")
  public double getPivotPositionRadians() {
    double motorRot = pivotPosition.getValueAsDouble();
    double outRot = motorRot / PIVOT_GEAR_RATIO;
    return Units.rotationsToRadians(outRot);
  }

  @AutoLogOutput(key = "Intake/Pivot/VelocityRadPerSec")
  public double getPivotVelocityRadPerSec() {
    double motorRps = pivotVelocity.getValueAsDouble();
    double outRps = motorRps / PIVOT_GEAR_RATIO;
    return Units.rotationsToRadians(outRps);
  }

  public void setRollerVoltage(double volts) {
    double clamped = MathUtil.clamp(volts, -MAX_ROLLER_VOLTS, MAX_ROLLER_VOLTS);
    rollerCommandVolts = clamped;
    rollerCommandPercent = clamped / 12.0;
    rollerMotor.setControl(rollerVoltageRequest.withOutput(clamped));
  }

  public void setRollerPercent(double percent) {
    double clamped = MathUtil.clamp(percent, -1.0, 1.0);
    rollerCommandPercent = clamped;
    rollerCommandVolts = clamped * 12.0;
    setRollerVoltage(rollerCommandVolts);
  }

  /** Run rollers to pull game piece in. Positive direction by convention. */
  public void runRollersIn(double percent) {
    setRollerPercent(Math.abs(percent));
  }

  /** Run rollers to push game piece out. Negative direction by convention. */
  public void runRollersOut(double percent) {
    setRollerPercent(-Math.abs(percent));
  }

  /** Convenience preset for first hardware tests (gentle intake). */
  public void intakeSlow() {
    runRollersIn(0.25);
  }

  /** Convenience preset for eject testing. */
  public void outtakeSlow() {
    runRollersOut(0.25);
  }

  public void stopRoller() {
    rollerCommandVolts = 0.0;
    rollerCommandPercent = 0.0;
    rollerMotor.stopMotor();
  }

  @AutoLogOutput(key = "Intake/Roller/VelocityRadPerSec")
  public double getRollerVelocityRadPerSec() {
    double motorRps = rollerVelocity.getValueAsDouble();
    double outRps = motorRps / ROLLER_GEAR_RATIO;
    return Units.rotationsToRadians(outRps);
  }

  public void stop() {
    stopPivot();
    stopRoller();
  }

  @Override
  public int[] getPowerPorts() {
    return powerPorts;
  }
}

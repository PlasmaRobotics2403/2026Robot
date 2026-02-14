package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.RBSISubsystem;

public class ShooterSubsystem extends RBSISubsystem {

  private static final int PIVOT_MOTOR_ID = 24;

  private final TalonFX pivotMotor;
  private final TalonFXConfiguration pivotConfig;

  public ShooterSubsystem() {
    pivotMotor = new TalonFX(PIVOT_MOTOR_ID, "rio");
    pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var pivotSlot0Configs = pivotConfig.Slot0;

    pivotSlot0Configs.kS = ShooterConstants.shooterPivotKS;
    pivotSlot0Configs.kV = ShooterConstants.shooterPivotKV;
    pivotSlot0Configs.kP = ShooterConstants.shooterPivotKP;
    pivotSlot0Configs.kD = ShooterConstants.shooterPivotKD;

    var pivotMotionMagicConfigs = pivotConfig.MotionMagic;

    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.shooterPivotVel; // rps
    pivotMotionMagicConfigs.MotionMagicAcceleration = ShooterConstants.shooterPivotAccel; // rps/s
    pivotMotionMagicConfigs.MotionMagicJerk = ShooterConstants.shooterPivotJerk; // rps/s/s

    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  @Override
  public void periodic() {}
}

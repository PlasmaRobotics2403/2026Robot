/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.wpiutils.AutoFeedEnable;
import com.ctre.phoenix6.wpiutils.CallbackHelper;
import com.ctre.phoenix6.wpiutils.MotorSafetyImplem;
import com.ctre.phoenix6.wpiutils.ReplayAutoEnable;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HAL.SimPeriodicBeforeCallback;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/** WPILib-integrated version of {@link CoreTalonFX}. */
public class TalonFX extends CoreTalonFX implements Sendable, AutoCloseable {
  /**
   * The StatusSignal getters are copies so that calls to the WPI interface do not update any
   * references
   */
  @SuppressWarnings("this-escape")
  private final StatusSignal<Double> m_dutyCycle = getDutyCycle(false).clone();

  private final DutyCycleOut m_setterControl = new DutyCycleOut(0);
  private final NeutralOut m_neutralControl = new NeutralOut();
  private final VoltageOut m_voltageControl = new VoltageOut(0);

  private final MotorOutputConfigs m_configs = new MotorOutputConfigs();

  private String m_description;

  private static final DeviceType kSimDeviceType = DeviceType.P6_TalonFXType;

  private SimDevice m_simMotor;
  private SimDouble m_simSupplyVoltage;
  private SimDouble m_simDutyCycle;
  private SimDouble m_simMotorVoltage;
  private SimDouble m_simTorqueCurrent;
  private SimDouble m_simSupplyCurrent;

  private SimDevice m_simForwardLimit;
  private SimBoolean m_simForwardLimitValue;

  private SimDevice m_simReverseLimit;
  private SimBoolean m_simReverseLimitValue;

  private SimDevice m_simRotor;
  private SimDouble m_simRotorPos;
  private SimDouble m_simRotorRawPos;
  private SimDouble m_simRotorVel;
  private SimDouble m_simRotorAccel;

  // returned registered callbacks
  private final ArrayList<CallbackStore> m_simValueChangedCallbacks =
      new ArrayList<CallbackStore>();
  private SimPeriodicBeforeCallback m_simPeriodicBeforeCallback = null;

  /** The default motor safety timeout IF calling application enables the feature. */
  public static final double kDefaultSafetyExpiration = 0.1;

  /**
   * Late-constructed motor safety, which ensures feature is off unless calling applications
   * explicitly enables it.
   */
  private MotorSafetyImplem m_motorSafety = null;

  private double m_motSafeExpiration = kDefaultSafetyExpiration;
  private final Lock m_motorSafetyLock = new ReentrantLock();

  /**
   * Constructs a new Talon FX motor controller object.
   *
   * <p>Constructs the device using the default CAN bus for the system (see {@link
   * CANBus#CANBus()}).
   *
   * @param deviceId ID of the device, as configured in Phoenix Tuner
   */
  public TalonFX(int deviceId) {
    this(deviceId, new CANBus());
  }

  /**
   * Constructs a new Talon FX motor controller object.
   *
   * @param deviceId ID of the device, as configured in Phoenix Tuner
   * @param canbus Name of the CAN bus this device is on. Possible CAN bus strings are:
   *     <ul>
   *       <li>"rio" for the native roboRIO CAN bus
   *       <li>CANivore name or serial number
   *       <li>SocketCAN interface (non-FRC Linux only)
   *       <li>"*" for any CANivore seen by the program
   *       <li>empty string (default) to select the default for the system:
   *           <ul>
   *             <li>"rio" on roboRIO
   *             <li>"can0" on Linux
   *             <li>"*" on Windows
   *           </ul>
   *     </ul>
   *
   * @deprecated Constructing devices with a CAN bus string is deprecated for removal in the 2027
   *     season. Construct devices using a {@link CANBus} instance instead.
   */
  @Deprecated(since = "2026", forRemoval = true)
  public TalonFX(int deviceId, String canbus) {
    this(deviceId, new CANBus(canbus));
  }

  /**
   * Constructs a new Talon FX motor controller object.
   *
   * @param deviceId ID of the device, as configured in Phoenix Tuner
   * @param canbus The CAN bus this device is on
   */
  @SuppressWarnings("this-escape")
  public TalonFX(int deviceId, CANBus canbus) {
    super(deviceId, canbus);

    m_description = "Talon FX (v6) " + deviceId;
    SendableRegistry.addLW(this, "Talon FX (v6) ", deviceId);

    if (RobotBase.isSimulation()) {
      /* run in both swsim and hwsim */
      AutoFeedEnable.getInstance().start();
    }
    if (Utils.isReplay()) {
      ReplayAutoEnable.getInstance().start();
    }

    m_simMotor = SimDevice.create("CANMotor:Talon FX (v6)", deviceId);

    final String base = "Talon FX (v6)[" + deviceId + "]/";
    m_simRotor = SimDevice.create("CANEncoder:" + base + "Rotor Sensor");
    m_simForwardLimit = SimDevice.create("CANDIO:" + base + "Fwd Limit");
    m_simReverseLimit = SimDevice.create("CANDIO:" + base + "Rev Limit");

    if (m_simMotor != null) {
      /* First make sure our simulated device gets enabled */
      m_simPeriodicBeforeCallback = HAL.registerSimPeriodicBeforeCallback(this::onPeriodic);

      m_simSupplyVoltage = m_simMotor.createDouble("supplyVoltage", Direction.kInput, 12.0);

      m_simDutyCycle = m_simMotor.createDouble("dutyCycle", Direction.kOutput, 0);
      m_simMotorVoltage = m_simMotor.createDouble("motorVoltage", Direction.kOutput, 0);
      m_simTorqueCurrent = m_simMotor.createDouble("torqueCurrent", Direction.kOutput, 0);
      m_simSupplyCurrent = m_simMotor.createDouble("supplyCurrent", Direction.kOutput, 0);

      final SimDeviceSim sim = new SimDeviceSim("CANMotor:Talon FX (v6)");
      m_simValueChangedCallbacks.add(
          sim.registerValueChangedCallback(m_simSupplyVoltage, this::onValueChanged, true));
    }
    if (m_simRotor != null) {
      m_simRotorPos = m_simRotor.createDouble("position", Direction.kOutput, 0);

      m_simRotorRawPos = m_simRotor.createDouble("rawPositionInput", Direction.kInput, 0);
      m_simRotorVel = m_simRotor.createDouble("velocity", Direction.kInput, 0);
      m_simRotorAccel = m_simRotor.createDouble("acceleration", Direction.kInput, 0);

      final SimDeviceSim sim = new SimDeviceSim("CANEncoder:" + base + "Rotor Sensor");
      m_simValueChangedCallbacks.add(
          sim.registerValueChangedCallback(m_simRotorRawPos, this::onValueChanged, true));
      m_simValueChangedCallbacks.add(
          sim.registerValueChangedCallback(m_simRotorVel, this::onValueChanged, true));
      m_simValueChangedCallbacks.add(
          sim.registerValueChangedCallback(m_simRotorAccel, this::onValueChanged, true));
    }
    if (m_simForwardLimit != null) {
      m_simForwardLimit.createBoolean("init", Direction.kOutput, true);
      m_simForwardLimit.createBoolean("input", Direction.kOutput, true);

      m_simForwardLimitValue = m_simForwardLimit.createBoolean("value", Direction.kBidir, false);

      final SimDeviceSim sim = new SimDeviceSim("CANDIO:" + base + "Fwd Limit");
      m_simValueChangedCallbacks.add(
          sim.registerValueChangedCallback(m_simForwardLimitValue, this::onValueChanged, true));
    }
    if (m_simReverseLimit != null) {
      m_simReverseLimit.createBoolean("init", Direction.kOutput, true);
      m_simReverseLimit.createBoolean("input", Direction.kOutput, true);

      m_simReverseLimitValue = m_simReverseLimit.createBoolean("value", Direction.kBidir, false);

      final SimDeviceSim sim = new SimDeviceSim("CANDIO:" + base + "Rev Limit");
      m_simValueChangedCallbacks.add(
          sim.registerValueChangedCallback(m_simReverseLimitValue, this::onValueChanged, true));
    }
  }

  // ----- Callbacks for Sim ----- //
  private void onValueChanged(String name, int handle, int direction, HALValue value) {
    String deviceName =
        SimDeviceDataJNI.getSimDeviceName(SimDeviceDataJNI.getSimValueDeviceHandle(handle));
    String physType = deviceName + ":" + name;
    PlatformJNI.JNI_SimSetPhysicsInput(
        kSimDeviceType.value, getDeviceID(), physType, CallbackHelper.getRawValue(value));
  }

  private void onPeriodic() {
    double value = 0;
    int err = 0;

    final int deviceID = getDeviceID();

    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "SupplyVoltage");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simSupplyVoltage.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "DutyCycle");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simDutyCycle.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "MotorVoltage");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simMotorVoltage.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "TorqueCurrent");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simTorqueCurrent.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "SupplyCurrent");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simSupplyCurrent.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "RotorPosition");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simRotorPos.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "RawRotorPosition");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simRotorRawPos.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "RotorVelocity");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simRotorVel.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "RotorAcceleration");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simRotorAccel.set(value);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "ForwardLimit");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simForwardLimitValue.set((int) value != 0);
    }
    value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "ReverseLimit");
    err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
    if (err == 0) {
      m_simReverseLimitValue.set((int) value != 0);
    }
  }

  // ------- AutoCloseable ----- //
  @Override
  public void close() {
    SendableRegistry.remove(this);
    if (m_simPeriodicBeforeCallback != null) {
      m_simPeriodicBeforeCallback.close();
      m_simPeriodicBeforeCallback = null;
    }
    if (m_simMotor != null) {
      m_simMotor.close();
      m_simMotor = null;
    }
    if (m_simRotor != null) {
      m_simRotor.close();
      m_simRotor = null;
    }
    if (m_simForwardLimit != null) {
      m_simForwardLimit.close();
      m_simForwardLimit = null;
    }
    if (m_simReverseLimit != null) {
      m_simReverseLimit.close();
      m_simReverseLimit = null;
    }

    for (var callback : m_simValueChangedCallbacks) {
      callback.close();
    }
    m_simValueChangedCallbacks.clear();

    AutoFeedEnable.getInstance().stop();
    ReplayAutoEnable.getInstance().stop();
  }

  // ------ set/get routines for WPILIB interfaces ------//
  /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public final void set(double speed) {
    feed();
    setControl(m_setterControl.withOutput(speed));
  }

  /**
   * Common interface for seting the direct voltage output of a motor controller.
   *
   * @param volts The voltage to output.
   */
  public final void setVoltage(double volts) {
    feed();
    setControl(m_voltageControl.withOutput(volts));
  }

  /**
   * Common interface for getting the current set speed of a motor controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  public final double get() {
    return m_dutyCycle.refresh().getValue();
  }

  // ---------Intercept CTRE calls for motor safety ---------//
  @Override
  protected StatusCode setControlPrivate(ControlRequest request) {
    /* intercept the control setter and feed motor-safety */
    feed();
    return super.setControlPrivate(request);
  }

  // ----------------------- turn-motor-off routines-------------------//
  /** Common interface for disabling a motor controller. */
  public final void disable() {
    setControl(m_neutralControl);
  }

  /** Common interface to stop motor movement until set is called again. */
  public final void stopMotor() {
    disable();
  }

  // -------------------- Neutral mode routines ----------------//
  /**
   * Sets the mode of operation when output is neutral or disabled. This is equivalent to setting
   * the {@link MotorOutputConfigs#NeutralMode} when applying a {@link TalonFXConfiguration} to the
   * motor.
   *
   * <p>Since neutral mode is a config, this API is blocking. We recommend that users avoid calling
   * this API periodically.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @param neutralMode The state of the motor controller bridge when output is neutral or disabled
   * @return Status of refreshing and applying the neutral mode config
   */
  public final StatusCode setNeutralMode(NeutralModeValue neutralMode) {
    return setNeutralMode(neutralMode, 0.100);
  }

  /**
   * Sets the mode of operation when output is neutral or disabled.
   *
   * <p>Since neutral mode is a config, this API is blocking. We recommend that users avoid calling
   * this API periodically.
   *
   * @param neutralMode The state of the motor controller bridge when output is neutral or disabled
   * @param timeoutSeconds Maximum amount of time to wait when performing configuration
   * @return Status of refreshing and applying the neutral mode config
   */
  public final StatusCode setNeutralMode(NeutralModeValue neutralMode, double timeoutSeconds) {
    /* First read the configs so they're up-to-date */
    StatusCode retval = getConfigurator().refresh(m_configs, timeoutSeconds);
    if (retval.isOK()) {
      /* Then set the neutral mode config to the appropriate value */
      m_configs.NeutralMode = neutralMode;
      retval = getConfigurator().apply(m_configs, timeoutSeconds);
    }
    return retval;
  }

  // ----- Sendable ----- //
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Value", this::get, this::set);
  }

  /**
   * @return Description of motor controller
   */
  public String getDescription() {
    return m_description;
  }

  /* ----- Motor Safety ----- */
  /** caller must lock appropriately */
  private MotorSafetyImplem GetMotorSafety() {
    if (m_motorSafety == null) {
      /* newly created MS object */
      m_motorSafety = new MotorSafetyImplem(this::stopMotor, getDescription());
      /* apply the expiration timeout */
      m_motorSafety.setExpiration(m_motSafeExpiration);
    }
    return m_motorSafety;
  }

  /**
   * Feed the motor safety object.
   *
   * <p>Resets the timer on this object that is used to do the timeouts.
   */
  public final void feed() {
    m_motorSafetyLock.lock();
    try {
      if (m_motorSafety == null) {
        /* do nothing, MS features were never enabled */
      } else {
        GetMotorSafety().feed();
      }
    } finally {
      m_motorSafetyLock.unlock();
    }
  }

  /**
   * Set the expiration time for the corresponding motor safety object.
   *
   * @param expirationTime The timeout value in seconds.
   */
  public final void setExpiration(double expirationTime) {
    m_motorSafetyLock.lock();
    try {
      /* save the value for if/when we do create the MS object */
      m_motSafeExpiration = expirationTime;
      /* apply it only if MS object exists */
      if (m_motorSafety == null) {
        /* do nothing, MS features were never enabled */
      } else {
        /* this call will trigger creating a registered MS object */
        GetMotorSafety().setExpiration(m_motSafeExpiration);
      }
    } finally {
      m_motorSafetyLock.unlock();
    }
  }

  /**
   * Retrieve the timeout value for the corresponding motor safety object.
   *
   * @return the timeout value in seconds.
   */
  public final double getExpiration() {
    m_motorSafetyLock.lock();
    try {
      /* return the intended expiration time */
      return m_motSafeExpiration;
    } finally {
      m_motorSafetyLock.unlock();
    }
  }

  /**
   * Determine of the motor is still operating or has timed out.
   *
   * @return a true value if the motor is still operating normally and hasn't timed out.
   */
  public final boolean isAlive() {
    m_motorSafetyLock.lock();
    try {
      if (m_motorSafety == null) {
        /* MC is alive - MS features were never enabled to neutral the MC. */
        return true;
      } else {
        return GetMotorSafety().isAlive();
      }
    } finally {
      m_motorSafetyLock.unlock();
    }
  }

  /**
   * Enable/disable motor safety for this device.
   *
   * <p>Turn on and off the motor safety option for this object.
   *
   * @param enabled True if motor safety is enforced for this object.
   */
  public final void setSafetyEnabled(boolean enabled) {
    m_motorSafetyLock.lock();
    try {
      if (m_motorSafety == null && !enabled) {
        /*
         * Caller wants to disable MS,
         * but MS features were nevere enabled,
         * so it doesn't need to be disabled.
         */
      } else {
        /* MS will be created if it does not exist */
        GetMotorSafety().setSafetyEnabled(enabled);
      }
    } finally {
      m_motorSafetyLock.unlock();
    }
  }

  /**
   * Return the state of the motor safety enabled flag.
   *
   * <p>Return if the motor safety is currently enabled for this device.
   *
   * @return True if motor safety is enforced for this device
   */
  public final boolean isSafetyEnabled() {
    m_motorSafetyLock.lock();
    try {
      if (m_motorSafety == null) {
        /* MS features were never enabled. */
        return false;
      } else {
        return GetMotorSafety().isSafetyEnabled();
      }
    } finally {
      m_motorSafetyLock.unlock();
    }
  }
}

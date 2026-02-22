# Agent Guidelines for FRC Robot Code Repository

## IMPORTANT: FRC-SPECIFIC KNOWLEDGE REQUIREMENT

**BEFORE CODING ANY ROBOT FEATURE, YOU MUST READ THE RELEVANT DOCUMENTATION.** This is an FRC (FIRST Robotics Competition) robot code repository using specialized robotics frameworks. Agents typically lack FRC knowledge, so you must study these frameworks first:

### **Core FRC Frameworks (MUST READ BEFORE CODING):**

1. **WPILib** (`docs/agent/WPILib/`) - Foundation of everything
   - **110+ RST files** covering robotics utilities, hardware APIs, control systems
   - **Purpose**: Core robotics library providing motor controllers, sensors, command framework, NetworkTables, kinematics, and advanced control systems

2. **AdvantageKit** (`docs/agent/AdvantageKit/`) - Robot logging framework
   - **42 markdown files** covering logging, replay, deterministic timestamps
   - **Purpose**: Advanced logging for deterministic replay and debugging with hardware abstraction via IO interfaces

3. **MapleSim** (`docs/agent/MapleSim/`) - Robot simulation library
   - **12 markdown files** covering physics engine integration with dyn4j
   - **Purpose**: Physics-based simulation replacing WPILib's simple simulation, providing realistic robot interactions and subsystem simulation

4. **PathPlanner** (`docs/agent/PathPlanner/`) - Autonomous path planning
   - **22+ markdown files** covering GUI and library usage
   - **Purpose**: Motion planning and autonomous path generation with pathfinding algorithms and trajectory following

5. **AdvantageScope** (`docs/agent/AdvantageScope/`) - External visualization tool
   - **36+ markdown files** covering data visualization and analysis
   - **Purpose**: Read robot logs and real-time telemetry with 2D/3D field visualization, data graphing, and analysis tools (users only, not agents)

6. **Phoenix6** (`docs/agent/Phoenix6/`) - CTRE hardware control library
   - **69+ RST files** covering motor controllers, sensors, and CAN bus communication
   - **Purpose**: Control CTRE hardware including TalonFX motors, Pigeon2 IMU, and CANCoder encoders via CAN bus

### **Command-based robot framework**

The robot uses WPILib's **command-based programming paradigm**, which structures robot code around reusable commands and subsystems. Understanding this framework is essential for modifying robot behavior:

**Boot Sequence:**
1. **Robot Initialization**: `Robot.java` extends `LoggedRobot` and initializes AdvantageKit logging
2. **RobotContainer Creation**: `RobotContainer.java` constructor runs, creating all subsystems and configuring commands
3. **Subsystem Registration**: All subsystems are registered with the `CommandScheduler`
4. **Command Binding**: Commands are bound to joystick buttons, triggers, and autonomous events
5. **Main Loop Entry**: Robot enters the main periodic loop

**Runtime Execution Flow:**
```
Robot Boots → RobotContainer Init → Subsystems Registered → Commands Bound
    ↓
Main Loop (50Hz/20ms period)
    ↓
Command Scheduler Runs (every cycle)
    ├── Executes scheduled commands
    ├── Updates subsystem periodic methods
    ├── Checks trigger conditions
    └── Schedules new commands when triggers activate
```

**Key Components:**
- **Subsystems**: Represent physical components (drive, intake, arm) with hardware control methods
- **Commands**: Represent actions (drive forward, shoot, climb) that use subsystems
- **Triggers**: Events (button presses, sensor readings, timers) that schedule commands
- **CommandScheduler**: Central coordinator that runs commands and updates subsystems

**Command Lifecycle:**
1. **Scheduled**: When a trigger activates or command is manually scheduled
2. **Initialized**: `initialize()` method runs once
3. **Executing**: `execute()` method runs every cycle (20ms)
4. **Ending**: `end()` method runs when command finishes (interrupted or completed)
5. **Finished**: Command removed from scheduler

**Common Patterns:**
- **Default Commands**: Run continuously when no other command requires the subsystem
- **Parallel Commands**: Multiple commands running simultaneously
- **Sequential Commands**: Commands running one after another
- **Conditional Commands**: Commands that run based on runtime conditions

**Important**: Always use the IO interface pattern for hardware abstraction to maintain simulation compatibility. Subsystems should only interact with hardware through IO interfaces, not directly with motor controllers or sensors.

### **CTRE Phoenix6 Framework Concepts**

The Phoenix6 library controls CTRE hardware via **CAN bus** (Controller Area Network). Key concepts:

**CAN Bus Communication:**
- Devices communicate over a shared CAN bus at 1Mbps
- Each device has a unique CAN ID (0-62 for standard, 0-63 for CANivore)
- Messages are prioritized by CAN ID (lower ID = higher priority)

**Status Signals:**
- Asynchronous data streams from hardware (position, velocity, faults)
- Use `getStatusSignal()` to subscribe to signals
- Signals update at hardware-defined rates (1ms-100ms)
- Must call `refresh()` or use `getValue()` with timeout

**Control Output:**
- Send control requests to hardware (position, velocity, duty cycle)
- Use `setControl()` with request objects
- Supports open-loop, closed-loop, motion profiling
- Requests are queued and executed by hardware

**Key CTRE Hardware Devices:**

1. **TalonFX Motor Controller**:
   - Brushless Falcon 500/ Kraken motor control
   - Integrated encoder (2048 CPR via Hall sensors)
   - Supports FOC (Field Oriented Control)
   - Built-in PID control with feedforward
   - Motion Magic for trapezoidal motion profiles
   - Current limiting and voltage compensation

2. **Pigeon2 IMU Module**:
   - 9-axis inertial measurement unit
   - Gyro, accelerometer, magnetometer
   - Yaw, pitch, roll measurements
   - Built-in sensor fusion
   - Mounting calibration
   - CAN bus or ribbon cable interface

3. **CANCoder Magnetic Encoder**:
   - Absolute magnetic position sensor
   - 4096 CPR resolution
   - Continuous rotation (no stops)
   - Magnet mounting calibration
   - Duty cycle or PWM output options
   - CAN bus communication

**Common Patterns:**
- Use `TalonFXConfiguration` for motor setup
- Configure `Slot0` for PID gains
- Set `NeutralMode` to `Brake` or `Coast`
- Enable `StatorCurrentLimit` for protection
- Use `StatusSignal` for asynchronous updates
- Implement hardware abstraction via IO interfaces

## Build/Lint/Test Commands

### **Build System**
- **Gradle-based** with WPILib GradleRIO plugin (version 2026.2.1)
- **Java 17** compatibility (sourceCompatibility = targetCompatibility = JavaVersion.VERSION_17)

### **Essential Commands**
```bash
# Build the project
./gradlew build

# Deploy to RoboRIO (real robot)
./gradlew deploy

# Run Java simulation (set Constants.currentRobotMode to SIM first)
./gradlew simulateJava

# Run tests (JUnit 5)
./gradlew test

# Run specific test class
./gradlew test --tests "frc.robot.subsystems.drive.DriveTest"

# Format code (runs automatically before compile)
./gradlew spotlessApply

# Clean build
./gradlew clean

# Build without tests
./gradlew assemble

# Run replay watch (AdvantageKit)
./gradlew replayWatch
```

### **Simulation Setup**
1. Change `Constants.currentRobotMode` to `Mode.SIM` in `src/main/java/frc/robot/Constants.java`
2. Run `./gradlew simulateJava`
3. Connect from AdvantageScope using the simulation layout

## Code Style Guidelines

### **Formatting (Spotless)**
- **Java**: Palantir Java Format 2.39.0 (formatJavadoc=true)
- **Gradle**: Greclipse with 4-space indentation
- **JSON**: Gson with 2-space indentation
- **Markdown**: 2-space indentation, trim trailing whitespace
- **Automatic formatting** runs before compilation (`project.compileJava.dependsOn(spotlessApply)`)

### **Imports**
```java
// Group imports in this order:
1. edu.wpi.first.* (WPILib)
2. org.littletonrobotics.* (AdvantageKit)
3. com.ctre.* (CTRE/Phoenix)
4. com.pathplanner.* (PathPlanner)
5. Other vendor libraries
6. Java standard library
7. Project packages (frc.robot.*)
```

### **Naming Conventions**
- **Classes**: PascalCase (e.g., `DriveCommands`, `ModuleIOTalonFX`)
- **Methods**: camelCase (e.g., `getPosition`, `setReference`)
- **Variables**: camelCase (e.g., `driveMotor`, `steerEncoder`)
- **Constants**: UPPER_SNAKE_CASE (e.g., `MAX_VELOCITY`, `MODULE_OFFSETS`)
- **Packages**: lowercase (e.g., `frc.robot.subsystems.drive`)

### **Types and Error Handling**
- Use `@NonNull` and `@Nullable` annotations where appropriate
- Prefer `Optional` over null returns
- Log errors using AdvantageKit's `Logger` instead of System.out
- Use WPILib's `Supplier` and `Consumer` interfaces for hardware abstraction

### **Hardware Abstraction Pattern**
This project uses the **IO Interface pattern** for simulation support:
```java
// Real implementation
public class ModuleIOTalonFX implements ModuleIO

// Simulation implementation
public class ModuleIOSim implements ModuleIO

// Usage in subsystem
private final ModuleIO[] modules;
```

## Project Structure

### **Key Directories**
```
src/main/java/frc/robot/
├── subsystems/
│   ├── drive/           # Swerve drive implementation
│   │   ├── Drive.java              # Main drive subsystem
│   │   ├── Module.java             # Individual swerve module
│   │   ├── ModuleIO.java           # Interface for hardware abstraction
│   │   ├── ModuleIOTalonFX.java    # Real hardware implementation
│   │   ├── ModuleIOSim.java        # Simulation implementation
│   │   ├── GyroIO.java             # Gyroscope interface
│   │   ├── GyroIOPigeon2.java      # Pigeon 2 implementation
│   │   └── GyroIOSim.java          # Gyro simulation
│   └── vision/          # Vision processing
│       ├── Vision.java             # Main vision subsystem
│       ├── VisionIO.java           # Vision interface
│       ├── VisionIOPhotonVision.java  # PhotonVision implementation
│       └── VisionIOLimelight.java  # Limelight implementation
├── commands/
│   └── DriveCommands.java  # Drive-related commands
├── util/
│   ├── PhoenixUtil.java    # CTRE/Phoenix utilities
│   └── LocalADStarAK.java  # Pathfinding utilities
├── generated/
│   └── TunerConstants.java # Auto-generated from Phoenix Tuner
├── Robot.java              # Main robot class
├── RobotContainer.java     # Robot container (subsystems, commands)
├── Constants.java          # Runtime mode constants
└── Main.java              # Entry point
```

### **Vendor Dependencies** (`vendordeps/`)
1. `AdvantageKit.json` - Logging and replay framework
2. `maple-sim.json` - Physics simulation engine
3. `PathplannerLib.json` - Path planning library
4. `Phoenix6.json` - CTRE motor controllers (Talon FX)
5. `photonlib.json` - Vision processing (PhotonVision)
6. `Studica.json` - Additional vendor library
7. `WPILibNewCommands.json` - WPILib command framework

## FRC-Specific Concepts

### **Robot Modes**
```java
public static enum Mode {
    REAL,    // Running on real robot
    SIM,     // Running physics simulator (MapleSim)
    REPLAY   // Replaying from log file (AdvantageKit)
}
```

### **Command-Based Programming**
- Uses WPILib's command framework
- Commands are composed in `RobotContainer`
- Use `CommandScheduler` for command execution

### **NetworkTables**
- WPILib's communication system for robot-to-driver station
- Use `NetworkTableInstance` for publishing/subscribing data

### **Swerve Drive Mathematics**
- Uses `SwerveDriveKinematics` and `SwerveDriveOdometry`
- Module states include `angle` and `speed`
- Field-relative vs robot-relative driving

## Documentation Index (Must Read Before Coding)

### **WPILib Documentation** (`docs/agent/WPILib/`) - **110+ files**
- `what-is-wpilib.rst` - Introduction to WPILib, supported languages, and source code
- `frc-glossary.rst` - FRC terminology glossary
- **Advanced Controls (47 files)**: Control theory, PID, feedforward, state-space control, system identification, trajectories
  - `advanced-controls/controllers/` - PID, feedforward, bang-bang, profiled PID controllers
  - `advanced-controls/filters/` - Debouncer, linear, median, slew rate limiter filters
  - `advanced-controls/state-space/` - State-space control, observers, pose estimators
  - `advanced-controls/system-identification/` - System identification routines and analysis
  - `advanced-controls/trajectories/` - Trajectory generation, constraints, Ramsete controller
- **Command-Based Programming (13 files)**: Framework for structuring robot code
  - `commandbased/what-is-command-based.rst` - Introduction to command-based programming
  - `commandbased/subsystems.rst` - Subsystem design and implementation
  - `commandbased/commands.rst` - Command creation and lifecycle
  - `commandbased/command-scheduler.rst` - Command scheduler operation
  - `commandbased/binding-commands-to-triggers.rst` - Trigger-based command scheduling
- **Hardware APIs (24 files)**: Motor controllers, sensors, pneumatics, LEDs
  - `hardware-apis/motors/` - PWM controllers, servos, motor controller usage
  - `hardware-apis/sensors/` - Accelerometers, encoders, gyros, limit switches, ultrasonics
  - `hardware-apis/pneumatics/` - Solenoids, pressure sensors
  - `hardware-apis/misc/addressable-leds.rst` - Addressable LED control
- **Kinematics and Odometry (7 files)**: Robot movement mathematics
  - `kinematics-and-odometry/swerve-drive-kinematics.rst` - Swerve drive kinematics mathematics
  - `kinematics-and-odometry/swerve-drive-odometry.rst` - Swerve drive odometry implementation
  - `kinematics-and-odometry/differential-drive-kinematics.rst` - Differential drive kinematics
  - `kinematics-and-odometry/mecanum-drive-kinematics.rst` - Mecanum drive kinematics
- **NetworkTables (10 files)**: Communication system for robot-to-driver station
  - `networktables/networktables-intro.rst` - NetworkTables introduction
  - `networktables/publish-and-subscribe.rst` - Publishing and subscribing to data
  - `networktables/robot-program.rst` - Integrating NetworkTables with robot code
- **Vision Processing**: Camera integration, AprilTags, vision processing pipelines

### **AdvantageKit Documentation** (`docs/agent/AdvantageKit/`) - **42 files**
- `getting-started/what-is-advantagekit/index.md` - Introduction to AdvantageKit's logging framework
- `theory/log-replay-comparison.md` - Detailed comparison of log replay techniques
- `data-flow/built-in-logging.md` - Built-in logging capabilities for common data types
- `data-flow/recording-inputs/io-interfaces.md` - Hardware abstraction pattern using IO interfaces
- `data-flow/recording-outputs/annotation-logging.md` - Annotation-based output logging
- `getting-started/template-projects/talonfx-swerve-template.md` - This specific template project
- `getting-started/installation/index.md` - Installation and setup instructions
- `getting-started/common-issues/multithreading.md` - Handling multithreading issues
- `theory/case-studies/` - Real-world examples including aiming functions, AprilTag tuning, auto-scoring
- `theory/deterministic-timestamps.md` - Ensuring deterministic timing for replay
- `theory/high-frequency-odometry.md` - High-frequency odometry techniques

### **MapleSim Documentation** (`docs/agent/MapleSim/`) - **12 files**
- `index.md` - Overview of maple-sim physics engine and its integration with dyn4j
- `installing-maple-sim.md` - Installation instructions for the maple-sim library
- `swerve-simulation-overview.md` - Documents on simulating a swerve drive train, covering kinematics and physics integration
- `swerve-sim-hardware-abstraction.md` - Essential document explaining how to implement hardware abstraction for swerve drive simulation
- `swerve-sim-easy.md` - Simplified approach to swerve simulation setup
- `simulation-details.md` - In-depth simulation details including physics engine configuration
- `using-the-simulated-arena.md` - Using the simulated field/arena with obstacles and game elements
- `simulating-intake.md` - Simulating intake subsystems with physics-based object interaction
- `simulating-projectiles.md` - Projectile simulation for shooting mechanisms
- `simulating-opponent-robots.md` - Simulating opponent robots for strategy testing
- `rebuilt.md` - Documentation on rebuilt/reconstructed simulation components
- `CONTRIBUTION.md` - Contribution guidelines for the maple-sim project

### **PathPlanner Documentation** (`docs/agent/PathPlanner/`) - **22+ files**
- `Home.md` - Overview and main features of PathPlanner
- `gui-Getting-Started.md` - GUI introduction for creating and editing paths
- `pplib-Getting-Started.md` - Library introduction for integrating PathPlanner into robot code
- `pplib-Build-an-Auto.md` - Building autonomous routines with path following and event markers
- `pplib-Pathfinding.md` - Pathfinding algorithms for navigating around obstacles
- `pplib-Swerve-Setpoint-Generator.md` - Swerve drive setpoint generation for smooth path following
- `pplib-Triggers.md` - Trigger system for executing commands during autonomous paths
- `pplib-Named-Commands.md` - Named commands for reusable autonomous actions
- `pplib-Override-Feedback.md` - Override feedback for manual control during autonomous
- `gui-Editing-Paths-and-Autos.md` - Editing paths and autonomous routines in the GUI
- `gui-Navigation-Grid-Editor.md` - Navigation grid editor for obstacle avoidance
- `gui-Telemetry.md` - Telemetry features for monitoring path execution
- `Robot-Config.md` - Robot configuration for PathPlanner integration

### **AdvantageScope Documentation** (`docs/agent/AdvantageScope/`) - **36+ files**
- `index.md` - Welcome and overview of AdvantageScope features
- `tab-reference/` - Visualization tabs including 3D field, line graphs, swerve visualization, console, joysticks
  - `tab-reference/3d-field/index.md` - 3D field visualization with robot models
  - `tab-reference/line-graph/index.md` - Line graph visualization for time-series data
  - `tab-reference/swerve.md` - Swerve drive module visualization
  - `tab-reference/console.md` - Console message viewer
- `overview/installation.md` - Installation instructions for AdvantageScope
- `overview/live-sources/` - Connecting to live robot data sources
  - `overview/live-sources/nt-publishing.md` - NetworkTables publishing configuration
  - `overview/live-sources/phoenix-diagnostics.md` - CTRE Phoenix motor controller diagnostics
  - `overview/live-sources/tuning-mode.md` - Tuning mode for parameter adjustment
- `overview/log-files/index.md` - Log file loading and analysis
- `more-features/custom-assets/` - Custom 3D asset creation and import
- `more-features/coordinate-systems.md` - Coordinate system configuration
- `more-features/urcl.md` - Universal Robot Control Language support

### **Phoenix6 Documentation** (`docs/agent/Phoenix6/`) - **69+ files**
- **API Reference (40+ files)**: Core library usage and device-specific APIs
  - `api-reference/api-usage/` - General API usage patterns
    - `api-overview.rst` - Phoenix6 API architecture overview
    - `status-signals.rst` - Asynchronous data streaming via status signals
    - `control-requests.rst` - Sending control commands to hardware
    - `configuration.rst` - Device configuration and persistence
    - `faults.rst` - Error handling and fault reporting
    - `signal-logging.rst` - Data logging and telemetry
  - `api-reference/device-specific/talonfx/` - TalonFX motor controller APIs
    - `talonfx-control-intro.rst` - Introduction to TalonFX control
    - `basic-pid-control.rst` - PID control implementation
    - `motion-magic.rst` - Motion Magic trapezoidal profiles
    - `open-loop-requests.rst` - Open-loop voltage/duty cycle control
    - `closed-loop-requests.rst` - Closed-loop position/velocity control
    - `remote-sensors.rst` - External sensor integration
  - `api-reference/mechanisms/` - Pre-built mechanism implementations
    - `swerve/` - Swerve drive APIs and simulation
      - `swerve-overview.rst` - Swerve drive system overview
      - `swerve-builder-api.rst` - Builder pattern for swerve configuration
      - `swerve-requests.rst` - Swerve-specific control requests
      - `swerve-simulation.rst` - Swerve drive simulation
      - `using-swerve-api.rst` - Practical swerve API usage
    - `differential/` - Differential drive mechanisms
      - `differential-overview.rst` - Differential drive overview
      - `differential-setup.rst` - Differential mechanism setup
      - `differential-tuning.rst` - PID tuning for differential drives
      - `using-differential-mech.rst` - Using differential mechanisms
  - `api-reference/simulation/` - Hardware simulation
    - `simulation-intro.rst` - Introduction to Phoenix6 simulation
  - `api-reference/wpilib-integration/` - WPILib framework integration
    - `motorcontroller-integration.rst` - MotorController class integration
    - `sysid-integration/` - System identification integration
      - `plumbing-and-running-sysid.rst` - SysId setup and execution
    - `unit-testing.rst` - Unit testing with Phoenix6
    - `epilogue-integration.rst` - Epilogue logging integration
- **Hardware Reference (10 files)**: Device specifications and capabilities
  - `hardware-reference/talonfx.rst` - TalonFX motor controller specifications
  - `hardware-reference/talonfxs.rst` - TalonFX (small) specifications
  - `hardware-reference/pigeon2.rst` - Pigeon2 IMU specifications and usage
  - `hardware-reference/cancoder.rst` - CANCoder magnetic encoder specifications
  - `hardware-reference/candle.rst` - CANdle LED controller
  - `hardware-reference/canrange.rst` - CANrange distance sensor
  - `hardware-reference/candi.rst` - CANdi device interface
  - `hardware-reference/improving-performance-with-current-limits.rst` - Performance optimization
  - `hardware-reference/pigeon-issues.rst` - Pigeon IMU troubleshooting
- **CANivore (5 files)**: High-speed CAN bus system
  - `canivore/canivore-intro.rst` - CANivore system introduction
  - `canivore/canivore-api.rst` - CANivore API usage
  - `canivore/canivore-config.rst` - CANivore configuration
  - `canivore/canivore-setup.rst` - CANivore hardware setup
  - `canivore/canivore-hardware-attached.rst` - Attached hardware management
- **Migration (8 files)**: Upgrading from previous versions
  - `migration/new-to-phoenix.rst` - New user introduction
  - `migration/canbus-utilization.rst` - CAN bus utilization guidelines
  - `migration/migration-guide/` - Migration from Phoenix 5
    - `api-structure-guide.rst` - API structure changes
    - `configuration-guide.rst` - Configuration migration
    - `control-requests-guide.rst` - Control request changes
    - `status-signals-guide.rst` - Status signal migration
    - `closed-loop-guide.rst` - Closed-loop control migration
    - `feature-replacements-guide.rst` - Feature replacement guide
- **Troubleshooting (3 files)**: Debugging and diagnostics
  - `troubleshooting/canbus-troubleshooting.rst` - CAN bus issues
  - `troubleshooting/running-diagnostics.rst` - Diagnostic procedures
- **Licensing (3 files)**: Software licensing information
  - `licensing/what-is-licensing.rst` - Licensing overview
  - `licensing/licensing.rst` - License details
  - `licensing/team-licensing.rst` - Team licensing options
- `support.rst` - Technical support resources

## Development Workflow

### **1. Before Making Changes**
- Read relevant framework documentation
- Understand the hardware abstraction pattern
- Check existing implementations for patterns

### **2. When Adding New Hardware**
- Create IO interface (`XxxIO.java`)
- Create real implementation (`XxxIOHardware.java`)
- Create simulation implementation (`XxxIOSim.java`)
- Update subsystem to use the interface

### **3. When Adding New Subsystem**
- Follow existing subsystem patterns
- Use AdvantageKit logging (`Logger.recordOutput()`)
- Implement simulation support via IO interface
- Add to `RobotContainer`

### **4. Testing**
- Run `./gradlew test` after changes
- Test in simulation mode (`Mode.SIM`)
- Verify logging works with AdvantageScope

### **5. Code Review Checklist**
- [ ] Follows hardware abstraction pattern
- [ ] Includes simulation implementation
- [ ] Uses AdvantageKit logging
- [ ] Follows naming conventions
- [ ] Properly formatted (Spotless)
- [ ] Documentation updated if needed

## Common Pitfalls

1. **Missing simulation support** - Always implement `XxxIOSim`
2. **Hardcoding hardware values** - Use constants or configuration
3. **Not using AdvantageKit logging** - Use `Logger` for all important data
4. **Ignoring thread safety** - WPILib has specific threading requirements
5. **Not testing in simulation** - Always test with `Mode.SIM` before real hardware

## Team Information
- **Team Number**: 5516 (from `.wpilib/wpilib_preferences.json`)
- **Project Year**: 2026
- **Template**: AdvantageKit Talon Swerve Template with maple-sim
- **Original Project**: Mechanical Advantage/AdvantageKit

## Additional Resources
- [AdvantageKit Online Documentation](https://docs.advantagekit.org)
- [WPILib Documentation](https://docs.wpilib.org)
- [CTRE Phoenix Documentation](https://docs.ctr-electronics.com)
- [PathPlanner Documentation](https://pathplanner.dev)

**REMEMBER**: This is REAL ROBOT CODE that will control physical hardware. Safety and reliability are paramount. Always test in simulation first, understand the physics implications, and follow FRC best practices.

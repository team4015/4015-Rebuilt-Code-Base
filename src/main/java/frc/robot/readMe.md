This is the code base for this year's robot
# Robot Code Base

This repository now includes a simulation + telemetry path that works directly with **AdvantageScope** for validating the swerve implementation.

## What is already wired in code

- WPILib datalogging starts automatically in `Robot.robotInit()`.
- Swerve simulation fallback is enabled when running in simulation (no hardware required).
- The following AdvantageScope-compatible NetworkTables topics are published continuously:
  - `AdvantageScope/Drive/Pose` (`Pose2d` struct)
  - `AdvantageScope/Drive/ModuleStates` (`SwerveModuleState[]` struct array)

## One-time setup on your computer

1. Install WPILib 2026 tools (includes sim + NT tools).
2. Install AdvantageScope (latest stable).
3. Clone this repo and run from the project root.

## Run the robot simulation

```bash
./gradlew simulateJava
```

That launches the robot program in simulation mode and publishes telemetry over NT4.

## Connect AdvantageScope live (NT4)

1. Open AdvantageScope.
2. Add a **Swerve** visualization.
3. For pose topic, select: `AdvantageScope/Drive/Pose`.
4. For module states topic, select: `AdvantageScope/Drive/ModuleStates`.
5. Set the NT source/host to:
   - `localhost` if sim + AdvantageScope are on the same machine.
   - your dev machine IP if remote.

## Logging + replay workflow (most accurate debugging loop)

1. Start simulation:
   ```bash
   ./gradlew simulateJava
   ```
2. Drive the robot in sim (joystick input / command input).
3. Stop sim.
4. Open the generated `.wpilog` file in AdvantageScope for frame-by-frame analysis.

Tip: In AdvantageScope, compare:
- module azimuth alignment vs commanded direction,
- wheel speed saturation behavior,
- odometry trajectory smoothness during rotation + translation.

## Notes on accuracy

- The simulation path models module kinematics and heading integration for software validation.
- Real-world effects (wheel scrub, slip, latency, electrical brownout, friction nonlinearities) still require on-robot tuning.
- Use this flow for algorithm validation first, then tune on carpet.
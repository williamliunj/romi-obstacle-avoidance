# Romi Wall Following

This is a Java/WPILib project for a Romi differential-drive robot. The robot can be driven manually with a joystick and can run an autonomous wall-following routine using two analog Sharp distance sensors.

## Current autonomous behavior

[Robot Run Demo](https://1drv.ms/v/c/748aa1d8a52fe4e5/IQBauUOpu4u9QbhEuSJpILC7AZGRVywDupI8qIcBgCYCOOk?e=rvpXEh)

`AutonomousDistance` is the default autonomous command. In the current version, it begins wall following immediately:

1. Drive forward at `0.5` speed.
2. Read both analog distance-sensor voltages continuously.
3. If sensor 1 reaches `2.4 V`, turn right by approximately 70 degrees.
4. If sensor 2 reaches `0.8 V` or higher, turn right by approximately 15 degrees.
5. If sensor 2 reaches `0.4 V` or lower, turn left by approximately 15 degrees.
6. Otherwise, continue driving straight.
7. After each turn, drive straight for a 0.5-second cooldown before checking the sensors again.
8. Stop after accumulating 370 inches of forward wall-following travel.

Turning movement is excluded from the 370-inch total. Sensor 1 must drop below `2.4 V` before it can trigger another 70-degree turn, preventing one continuous high reading from causing repeated turns.

> **Safety:** The autonomous routine has no time-based safety timeout. Keep the robot supervised and be ready to disable it during testing.

## Hardware configuration

- Pololu Romi chassis with Romi 32U4 control board
- Raspberry Pi running the Romi WPILib service
- Left motor: PWM channel 0
- Right motor: PWM channel 1
- Left encoder: DIO 4/5
- Right encoder: DIO 6/7
- Sensor 1: Analog Input 0 / EXT1
- Sensor 2: Analog Input 1 / EXT2
- Sharp GP2Y0A02YK0F analog distance sensor used by the project

The distance sensors currently provide raw voltages. The code compares those measured voltages directly rather than converting them to centimeters or inches.

## Drivetrain details

`Drivetrain.java` controls both motors, reads the encoders and sensors, and exposes gyro and accelerometer values.

- Wheel diameter: `2.75591` inches (70 mm)
- Encoder counts per revolution: `1440`
- Effective track width used for turns: `5.551` inches
- Right motor output factor: `0.96`
- Right motor is inverted so both wheels drive forward together
- Sensor voltages are printed to the program terminal every 500 ms

On this robot, negative arcade-drive rotation turns right and positive rotation turns left.

## Project structure

```text
src/main/java/frc/robot/
├── Main.java
├── Robot.java
├── RobotContainer.java
├── Constants.java
├── subsystems/
│   └── Drivetrain.java
└── commands/
    ├── ArcadeDrive.java
    ├── AutonomousDistance.java
    ├── AutonomousTime.java
    ├── DriveDistance.java
    ├── DriveTime.java
    ├── DriveUntilObstacle.java
    ├── ObstacleAvoidanceStep.java
    ├── TurnDegrees.java
    └── TurnTime.java
```

`RobotContainer.java` creates the drivetrain, configures joystick control, and places the two autonomous choices on SmartDashboard. `Auto Routine Distance` is selected by default.

`DriveUntilObstacle.java` and `ObstacleAvoidanceStep.java` are currently placeholders for future development. `DriveUntilObstacle` also contains a helper that creates a 70-degree right-turn command.

## Requirements

- WPILib 2026
- Java 17
- A configured Romi WPILib environment
- A joystick or gamepad on USB port 0 for manual control

The project uses GradleRIO `2026.2.1` and includes desktop simulation support.

## Build and run

Connect the computer to the Romi network and make sure the Romi WPILib service is running. From the repository directory, compile with:

```bash
./gradlew compileJava
```

To launch the WPILib desktop simulation/Romi program:

```bash
./gradlew simulateJava
```

Use the Driver Station to enable autonomous mode. The sensor readings appear in the program terminal in this format:

```text
Sensor 1: 0.000 V | Sensor 2: 0.000 V
```

## Important tuning values

The main autonomous settings are near the top of `AutonomousDistance.java`:

| Setting | Current value | Purpose |
|---|---:|---|
| `kForwardSpeed` | `0.5` | Straight driving speed |
| `kTurnSpeed` | `0.5` | Turning speed |
| `kTargetTravelDistanceInches` | `370.0` | Forward distance before stopping |
| `kSensor1TurnVoltage` | `2.4 V` | Triggers a 70-degree right turn |
| `kSensor2HighVoltage` | `0.8 V` | Triggers a right correction |
| `kSensor2LowVoltage` | `0.4 V` | Triggers a left correction |
| `kCorrectionDegrees` | `15.0°` | Size of sensor-2 corrections |
| `kSensorCheckCooldownSeconds` | `0.5 s` | Straight-driving time after a turn |

These voltage thresholds were calibrated for the installed sensors and environment. Recheck them if the sensor mounting, wall material, lighting, or power supply changes.

## License

This project uses WPILib code under the BSD license. See [WPILib-License.md](WPILib-License.md).

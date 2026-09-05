// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Drives until sensor 1 reaches 2.4 volts, then runs the 70-degree turn command.
   * The approach has no timeout.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
        new FunctionalCommand(
            // Initialize: start with the motors stopped.
            () -> drivetrain.arcadeDrive(0, 0),
            // Execute: check sensor 1 on every scheduler cycle.
            () -> {
              if (drivetrain.getDistanceSensorVoltage() >= 2.4) {
                drivetrain.arcadeDrive(0, 0);
              } else {
                drivetrain.arcadeDrive(0.5, 0);
              }
            },
            // End: stop on completion or interruption.
            interrupted -> drivetrain.arcadeDrive(0, 0),
            // Finish the approach so the turn can begin.
            () -> drivetrain.getDistanceSensorVoltage() >= 2.4,
            drivetrain),
        new DriveUntilObstacle().createTurnCommand(drivetrain),
        new WallFollowUntilObstacle(drivetrain));
  }

  /**
   * Drives forward while sensor 2 keeps the robot near the wall. The command
   * ends after 108 inches of forward wall-following travel.
   */
  private static class WallFollowUntilObstacle extends Command {
    private static final double kForwardSpeed = 0.5;
    private static final double kTurnSpeed = 0.5;
    private static final double kTargetTravelDistanceInches = 300.0;
    private static final double kSensor1TurnVoltage = 2.4;
    private static final double kSensor2HighVoltage = 0.8;
    private static final double kSensor2LowVoltage = 0.4;
    private static final double kCorrectionDegrees = 15.0;
    private static final double kSensor1TurnDegrees = 70.0;
    private static final double kTrackWidthInches = 5.551;
    private static final double kSensorCheckCooldownSeconds = 0.5;

    private enum State {
      DRIVE_FORWARD,
      TURN_RIGHT,
      TURN_RIGHT_70_DEGREES,
      TURN_LEFT,
      DRIVE_DURING_COOLDOWN
    }

    private final Drivetrain m_drivetrain;
    private final Timer m_cooldownTimer = new Timer();
    private State m_state;
    private double m_traveledDistanceInches;
    private double m_lastForwardEncoderDistanceInches;
    private boolean m_sensor1ReadyToTrigger;

    WallFollowUntilObstacle(Drivetrain drivetrain) {
      m_drivetrain = drivetrain;
      addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
      m_state = State.DRIVE_FORWARD;
      m_cooldownTimer.stop();
      m_cooldownTimer.reset();
      m_traveledDistanceInches = 0.0;
      m_lastForwardEncoderDistanceInches = 0.0;
      m_sensor1ReadyToTrigger = true;
      m_drivetrain.resetEncoders();
      m_drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public void execute() {
      // Count forward travel, but do not include the distance used while turning.
      if (m_state == State.DRIVE_FORWARD || m_state == State.DRIVE_DURING_COOLDOWN) {
        recordForwardDistance();
      }

      if (m_state == State.DRIVE_FORWARD) {
        double sensor1Voltage = m_drivetrain.getDistanceSensorVoltage();
        double sensor2Voltage = m_drivetrain.getDistanceSensorVoltage2();

        if (sensor1Voltage < kSensor1TurnVoltage) {
          m_sensor1ReadyToTrigger = true;
        }

        if (m_sensor1ReadyToTrigger && sensor1Voltage >= kSensor1TurnVoltage) {
          m_sensor1ReadyToTrigger = false;
          startTurn(State.TURN_RIGHT_70_DEGREES);
        } else if (sensor2Voltage >= kSensor2HighVoltage) {
          startTurn(State.TURN_RIGHT);
        } else if (sensor2Voltage <= kSensor2LowVoltage) {
          startTurn(State.TURN_LEFT);
        } else {
          m_drivetrain.arcadeDrive(kForwardSpeed, 0);
        }
      } else if (m_state == State.TURN_RIGHT || m_state == State.TURN_RIGHT_70_DEGREES) {
        m_drivetrain.arcadeDrive(0, -kTurnSpeed);
        finishTurnWhenReady();
      } else if (m_state == State.TURN_LEFT) {
        m_drivetrain.arcadeDrive(0, kTurnSpeed);
        finishTurnWhenReady();
      } 
      else {
        // Move forward before allowing sensor 2 to request another correction.
        m_drivetrain.arcadeDrive(kForwardSpeed, 0);
        if (m_cooldownTimer.hasElapsed(kSensorCheckCooldownSeconds)) {
          m_cooldownTimer.stop();
          m_state = State.DRIVE_FORWARD;
        }
      }
    }

    private void startTurn(State turnDirection) {
      m_drivetrain.arcadeDrive(0, 0);
      m_drivetrain.resetEncoders();
      m_lastForwardEncoderDistanceInches = 0.0;
      m_state = turnDirection;
    }

    private void recordForwardDistance() {
      double currentDistance =
          (Math.abs(m_drivetrain.getLeftDistanceInch())
                  + Math.abs(m_drivetrain.getRightDistanceInch()))
              / 2.0;
      double distanceSinceLastCheck = currentDistance - m_lastForwardEncoderDistanceInches;

      if (distanceSinceLastCheck > 0.0) {
        m_traveledDistanceInches += distanceSinceLastCheck;
      }
      m_lastForwardEncoderDistanceInches = currentDistance;
    }

    private void finishTurnWhenReady() {
      double leftDistance = Math.abs(m_drivetrain.getLeftDistanceInch());
      double rightDistance = Math.abs(m_drivetrain.getRightDistanceInch());
      double averageDistance = (leftDistance + rightDistance) / 2.0;
      double turnDegrees =
          m_state == State.TURN_RIGHT_70_DEGREES ? kSensor1TurnDegrees : kCorrectionDegrees;
      double targetDistance = Math.PI * kTrackWidthInches / 360.0 * turnDegrees;

      if (averageDistance >= targetDistance) {
        m_drivetrain.arcadeDrive(0, 0);
        // Start a fresh encoder segment so the turn does not count toward 108 inches.
        m_drivetrain.resetEncoders();
        m_lastForwardEncoderDistanceInches = 0.0;
        m_cooldownTimer.restart();
        m_state = State.DRIVE_DURING_COOLDOWN;
      }
    }

    @Override
    public boolean isFinished() {
      return m_traveledDistanceInches >= kTargetTravelDistanceInches;
    }

    @Override
    public void end(boolean interrupted) {
      m_cooldownTimer.stop();
      m_drivetrain.arcadeDrive(0, 0);
    }
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/** Placeholder for a future sensor-aware driving command. */
public class DriveUntilObstacle extends Command {
  public DriveUntilObstacle() {
    // Add command setup here later.
  }

  /** Creates a turn command; schedule it or add it to a sequence to run it. */
  public Command createTurnCommand(Drivetrain drivetrain) {
    return new TurnDegrees(-0.5, 70, drivetrain);
  }
}

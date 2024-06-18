package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.drive.Drive;

public final class AutoRoutines {
  public static final Command none() {
    return Commands.none();
  }

  public static final Command center_abc(Drive drive) {
    return CompositeCommands.getChoreoCommand(drive, "Center-ABC");
  }
}

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.AutoPathPoints;
import frc.robot.subsystems.drive.Drive;

public final class AutoRoutines {
  public static final PathConstraints SLOW_PATH_CONSTRAINTS = new PathConstraints(1, 1, 4, 4);

  public static final Command none() {
    return Commands.none();
  }

  public static final Command leaveAuto(Drive drive) {
    return CompositeCommands.getPath(
        drive, AutoPathPoints.OPPONENT_SOURCE_AGAINST_ALLIANCE_WALL, AutoPathPoints.OUT_OF_THE_WAY);
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public final class AutoRoutines {
  public static final Command none() {
    return Commands.none();
  }

  public static final Command test(Drive drive) {
    return CompositeCommands.getChoreoCommand(drive, "Test");
  }

  public static final Command center_abc(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Turret turret,
      Feeder feeder,
      Hood hood,
      Shooter shooter) {
    return Commands.parallel(
        CompositeCommands.getChoreoCommand(drive, "Amp-CBA"),
        CompositeCommands.getShootSpeakerCommand(
            intake, serializer, turret, feeder, hood, shooter));
  }
}

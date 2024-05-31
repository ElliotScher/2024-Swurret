package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
      new PathConstraints(5.0, 4.0, 540.0, 720.0);

  public static final Command resetHeading() {
    return Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    new Pose2d(RobotState.getRobotPose().getTranslation(), new Rotation2d())))
        .ignoringDisable(true);
  }

  public static final Command getPath(Pose2d endingPose) {
    return AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(endingPose), DEFAULT_PATH_CONSTRAINTS);
  }

  public static final Command getPath(Pose2d endingPose, PathConstraints pathConstraints) {
    return AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(endingPose), pathConstraints);
  }

  public static final Command getPath(Drive drive, Pose2d startingPose, Pose2d endingPose) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(AllianceFlipUtil.apply(startingPose));
              drive.setPose(AllianceFlipUtil.apply(startingPose));
            }),
        AutoBuilder.pathfindToPose(
            DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? endingPose
                : AllianceFlipUtil.apply(endingPose),
            DEFAULT_PATH_CONSTRAINTS));
  }

  public static final Command getPath(
      Drive drive, Pose2d startingPose, Pose2d endingPose, PathConstraints pathConstraints) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(AllianceFlipUtil.apply(startingPose));
              drive.setPose(AllianceFlipUtil.apply(startingPose));
            }),
        AutoBuilder.pathfindToPose(
            DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? endingPose
                : AllianceFlipUtil.apply(endingPose),
            pathConstraints));
  }
}

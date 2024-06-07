package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
      new PathConstraints(5.0, 4.0, 540.0, 720.0);

  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    drive,
                    new Pose2d(RobotState.getRobotPose().getTranslation(), new Rotation2d())))
        .ignoringDisable(true);
  }

  public static final Command getChoreoCommand(Drive drive, String trajectory) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    drive,
                    AllianceFlipUtil.apply(Choreo.getTrajectory(trajectory).getInitialPose()))),
        Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(trajectory),
            () -> RobotState.getRobotPose(),
            new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get()),
            new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get()),
            new PIDController(
                DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get()),
            (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
            true,
            drive));
  }

  public static final Command getPath(Pose2d endingPose) {
    return AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(endingPose), DEFAULT_PATH_CONSTRAINTS);
  }

  public static final Command getPath(Pose2d endingPose, PathConstraints pathConstraints) {
    return AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(endingPose), pathConstraints);
  }
}

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
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
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

  public static final Command getCollectCommand(
      Intake intake, Serializer serializer, Feeder feeder) {
    return Commands.parallel(
        intake.intake(() -> feeder.hasNote()),
        serializer.intake(() -> feeder.hasNote()),
        feeder.intake());
  }

  public static final Command getShootCommand(Intake intake, Serializer serializer, Feeder feeder) {
    return Commands.parallel(intake.shoot(), serializer.shoot(), feeder.shoot());
  }

  public static final Command runSpeakerTarget(Turret turret, Hood hood, Shooter shooter) {
    return Commands.parallel(
        turret.setShootPosition(), hood.setSpeakerPosition(), shooter.setSpeakerSpeed());
  }

  public static final Command runAmpTarget(Turret turret, Hood hood, Shooter shooter) {
    return Commands.parallel(turret.setAmpPosition(), hood.setAmpPosition(), shooter.setAmpSpeed());
  }

  public static final Command runFeedTarget(Turret turret, Hood hood, Shooter shooter) {
    return Commands.parallel(
        turret.setFeedPosition(), hood.setFeedPosition(), shooter.setFeedSpeed());
  }

  public static final Command getShootSpeakerCommand(
      Intake intake,
      Serializer serializer,
      Turret turret,
      Feeder feeder,
      Hood hood,
      Shooter shooter) {
    return Commands.parallel(
        runSpeakerTarget(turret, hood, shooter),
        getShootCommand(intake, serializer, feeder)
            .onlyIf(() -> RobotState.shooterReady(turret, hood, shooter)));
  }

  public static final Command getShootAmpCommand(
      Intake intake,
      Serializer serializer,
      Turret turret,
      Feeder feeder,
      Hood hood,
      Shooter shooter) {
    return Commands.parallel(
        runAmpTarget(turret, hood, shooter),
        getShootCommand(intake, serializer, feeder)
            .onlyIf(() -> RobotState.shooterReady(turret, hood, shooter)));
  }

  public static final Command getFeedCommand(
      Intake intake,
      Serializer serializer,
      Turret turret,
      Feeder feeder,
      Hood hood,
      Shooter shooter) {
    return Commands.parallel(
        runFeedTarget(turret, hood, shooter),
        getShootCommand(intake, serializer, feeder)
            .onlyIf(() -> RobotState.shooterReady(turret, hood, shooter)));
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

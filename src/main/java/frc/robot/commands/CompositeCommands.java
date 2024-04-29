package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMode;
import frc.robot.subsystems.vision.VisionPipeline;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;

public class CompositeCommands {
  public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
      new PathConstraints(5.0, 4.0, 540.0, 720.0);

  public static final Command getCollectCommand(
      Intake intake, Serializer serializer, Vision noteVision, Vision aprilTagVision) {
    return Commands.sequence(
            intake.deployIntake(),
            Commands.race(intake.runVoltage(), serializer.intake()),
            Commands.parallel(
                intake.retractIntake(), noteVision.blinkLEDs(), aprilTagVision.blinkLEDs()))
        .finallyDo(
            () -> {
              noteVision.disableLEDs();
              aprilTagVision.disableLEDs();
            });
  }

  public static final Command getCollectCommand(Intake intake, Serializer serializer) {
    return Commands.sequence(
        intake.deployIntake(),
        Commands.race(intake.runVoltage(), serializer.intake()),
        intake.retractIntake());
  }

  public static final Command getOuttakeCommand(
      Intake intake, Serializer serializer, Kicker kicker) {
    return Commands.parallel(intake.outtake(), serializer.outtake(), kicker.outtake());
  }

  public static final Command getSourceFeedCommand(
      Shooter shooter, Hood hood, Accelerator accelerator, Kicker kicker) {
    return shooter.runSourceFeed().alongWith(hood.setSourceFeed(), accelerator.runAccelerator());
  }

  public static final Command getAmpFeedCommand(
      Shooter shooter, Hood hood, Accelerator accelerator, Kicker kicker) {
    return shooter.runAmpFeed().alongWith(hood.setAmpFeed(), accelerator.runAccelerator());
  }

  public static final Command getPosePrepShooterCommand(
      Drive drive, Hood hood, Shooter shooter, Accelerator accelerator, Vision aprilTagVision) {
    return shooter
        .runPoseDistance(
            () -> RobotState.getRobotPose().getTranslation(), drive::getFieldRelativeVelocity)
        .alongWith(
            hood.setPosePosition(
                () -> RobotState.getRobotPose().getTranslation(), drive::getFieldRelativeVelocity))
        .alongWith(accelerator.runAccelerator());
  }

  public static final Command getShootCommand(Intake intake, Serializer serializer, Kicker kicker) {
    return serializer.shoot().alongWith(intake.runVoltage(), kicker.shoot()).withTimeout(0.25);
  }

  public static final Command getFeedCommand(Intake intake, Serializer serializer, Kicker kicker) {
    return Commands.sequence(
        intake.deployIntake(),
        Commands.parallel(intake.runVoltage(), serializer.intake(), kicker.shoot()));
  }

  public static final Command getTrackNoteCenterCommand(
      Drive drive, Intake intake, Serializer serializer, Vision noteVision, Vision aprilTagVision) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, (FieldConstants.fieldLength / 2.0) + 1, VisionMode.Notes)
            .raceWith(getCollectCommand(intake, serializer)))
        .withTimeout(3);
  }

  public static final Command getTrackNoteSpikeCommand(
      Drive drive, Intake intake, Serializer serializer, Vision noteVision, Vision aprilTagVision) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, FieldConstants.startingLineX + 0.5, VisionMode.Notes)
            .raceWith(getCollectCommand(intake, serializer)))
        .withTimeout(2);
  }

  public static final Command getTrackNoteSpikeCommand(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Vision noteVision,
      Vision aprilTagVision,
      double maxSpeed) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, FieldConstants.startingLineX + 1, VisionMode.Notes, maxSpeed)
            .raceWith(getCollectCommand(intake, serializer)))
        .withTimeout(2);
  }

  public static final Command getTrackSpeakerFarCommand(
      Drive drive, Hood hood, Shooter shooter, Vision aprilTagVision) {
    return DriveCommands.moveTowardsTarget(drive, aprilTagVision, 3.75, VisionMode.AprilTags);
  }

  public static final Command getTrackSpeakerCloseCommand(
      Drive drive, Hood hood, Shooter shooter, Vision aprilTagVision) {
    return DriveCommands.moveTowardsTarget(
        drive, aprilTagVision, FieldConstants.startingLineX + 0.05, VisionMode.AprilTags);
  }

  public static final Command getAimSpeakerCommand(Drive drive) {
    return DriveCommands.aimTowardsTarget(drive);
  }

  public static final Command shootOnTheMove(
      Drive drive, Intake intake, Serializer serializer, Kicker kicker, Vision vision) {
    return Commands.run(
            () -> {
              PPHolonomicDriveController.setRotationTargetOverride(
                  () ->
                      Optional.of(
                          RobotState.poseCalculation(drive.getFieldRelativeVelocity())
                              .robotAngle()));
            })
        .alongWith(getShootCommand(intake, serializer, kicker));
  }

  public static final Command getPath(Pose2d endingPose) {
    return AutoBuilder.pathfindToPose(
        DriverStation.getAlliance().get().equals(Alliance.Blue)
            ? endingPose
            : AllianceFlipUtil.apply(endingPose),
        DEFAULT_PATH_CONSTRAINTS);
  }

  public static final Command getPath(Pose2d endingPose, PathConstraints pathConstraints) {
    return AutoBuilder.pathfindToPose(
        DriverStation.getAlliance().get().equals(Alliance.Blue)
            ? endingPose
            : AllianceFlipUtil.apply(endingPose),
        pathConstraints);
  }

  public static final Command getPath(
      Drive drive,
      Vision noteVision,
      VisionPipeline noteTrackingPipeline,
      Pose2d startingPose,
      Pose2d endingPose) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              noteVision.setPipeline(noteTrackingPipeline);
              drive.setPose(
                  DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? startingPose
                      : AllianceFlipUtil.apply(startingPose));
            }),
        AutoBuilder.pathfindToPose(
            DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? endingPose
                : AllianceFlipUtil.apply(endingPose),
            DEFAULT_PATH_CONSTRAINTS));
  }

  public static final Command getPath(
      Drive drive,
      Vision noteVision,
      VisionPipeline noteTrackingPipeline,
      Pose2d startingPose,
      Pose2d endingPose,
      PathConstraints pathConstraints) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              noteVision.setPipeline(noteTrackingPipeline);
              drive.setPose(
                  DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? startingPose
                      : AllianceFlipUtil.apply(startingPose));
            }),
        AutoBuilder.pathfindToPose(
            DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? endingPose
                : AllianceFlipUtil.apply(endingPose),
            pathConstraints));
  }
}

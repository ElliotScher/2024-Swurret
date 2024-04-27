package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoPathPoints;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class AutoRoutines {
  public static final double MAX_SPEED_SPIKE_NOTE_TRACKING = 0.5;
  public static final PathConstraints SLOW_PATH_CONSTRAINTS = new PathConstraints(1, 1, 2, 2);

  private static final Command leaveAuto(Drive drive) {
    return CompositeCommands.getPath(
        drive, AutoPathPoints.OPPONENT_SOURCE_AGAINST_ALLIANCE_WALL, AutoPathPoints.OUT_OF_THE_WAY);
  }

  private static final Command gtfootw(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.OUT_OF_THE_WAY));
  }

  private static final Command sourceSideTwoPiece(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_PODIUM_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command ampSideTwoPiece(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.AMP_SIDE_SUBWOOFER, AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command centerTwoPiece(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.CENTER_SUBWOOFER, AutoPathPoints.CENTER_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command sourceSideWingAuto(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_PODIUM_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_2),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_3),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command ampSideWingAuto(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.AMP_SIDE_SUBWOOFER, AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_2),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_1),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command centerWingAutoStartAmp(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.CENTER_SUBWOOFER, AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_2),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_1),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command centerWingAutoStartSource(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.CENTER_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_PODIUM_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_2),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_3),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command uriAuto(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_PODIUM_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_4)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command uriPlus1Auto(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_PODIUM_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_4)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_5)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command midline54(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_4)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_5)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command midline45(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.SOURCE_SIDE_SUBWOOFER, AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_5)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_4)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command roboteersCompliment(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    DriverStation.getAlliance().get().equals(Alliance.Blue)
                        ? AutoPathPoints.CENTER_SUBWOOFER
                        : AllianceFlipUtil.apply(AutoPathPoints.CENTER_SUBWOOFER))),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_6)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.CENTER_SHOT_FAR),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(
                AutoPathPoints.SUBWOOFER_CENTER_DROPPED_NOTE, SLOW_PATH_CONSTRAINTS)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_PODIUM_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command roboteersComplimentMinus1(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    DriverStation.getAlliance().get().equals(Alliance.Blue)
                        ? AutoPathPoints.CENTER_SUBWOOFER
                        : AllianceFlipUtil.apply(AutoPathPoints.CENTER_SUBWOOFER))),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_6)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.CENTER_SHOT_FAR),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(
                AutoPathPoints.SUBWOOFER_CENTER_DROPPED_NOTE, SLOW_PATH_CONSTRAINTS)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.CENTER_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command ampSideThreePiece(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.AMP_SIDE_SUBWOOFER, AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_8)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command ampSideFourPiece(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
            drive, AutoPathPoints.AMP_SIDE_SUBWOOFER, AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, MAX_SPEED_SPIKE_NOTE_TRACKING),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_8)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_7)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.AMP_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command midline543(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
                drive, AutoPathPoints.OPPONENT_SOURCE_AGAINST_STARTING_LINE, AutoPathPoints.NOTE_4)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_5)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_6),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  private static final Command midline43(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    return Commands.sequence(
        CompositeCommands.getPath(
                drive, AutoPathPoints.OPPONENT_SOURCE_AGAINST_STARTING_LINE, AutoPathPoints.NOTE_5)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker),
        CompositeCommands.getPath(AutoPathPoints.NOTE_6)
            .alongWith(CompositeCommands.getCollectCommand(intake, serializer)),
        CompositeCommands.getPath(AutoPathPoints.SOURCE_SIDE_SHOT),
        CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision),
        CompositeCommands.getShootCommand(intake, serializer, kicker));
  }

  public static final Map<String, Command> autoList(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Kicker kicker,
      Vision aprilTagVision,
      Vision noteVision) {
    Map<String, Command> map = new HashMap<String, Command>();
    map.put("NONE", Commands.none());
    map.put(
        "SOURCE SIDE TWO PIECE",
        sourceSideTwoPiece(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "SOURCE SIDE FULL WING AUTO",
        sourceSideWingAuto(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put("URI Auto", uriAuto(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put("URI + 1", uriPlus1Auto(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "MIDLINE 5, 4, 3",
        midline543(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "MIDLINE 5, 4", midline54(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "MIDLINE 4, 5", midline45(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "MIDLINE 4, 3", midline43(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "AMP SIDE TWO PIECE",
        ampSideTwoPiece(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "AMP SIDE THREE PIECE",
        ampSideThreePiece(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "AMP SIDE FOUR PIECE",
        ampSideFourPiece(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "AMP SIDE FULL WING AUTO",
        ampSideWingAuto(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "CENTER TWO PIECE",
        centerTwoPiece(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "CENTER-AMP FULL WING AUTO",
        centerWingAutoStartAmp(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "CENTER-SOURCE FULL WING AUTO",
        centerWingAutoStartSource(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "ROBOTEER'S COMPLIMENT",
        roboteersCompliment(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put(
        "ROBOTEER'S COMPLIMENT - 1",
        roboteersComplimentMinus1(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put("GTFOOTW", gtfootw(drive, intake, serializer, kicker, aprilTagVision, noteVision));
    map.put("LEAVE", leaveAuto(drive));
    return map;
  }

  public static final List<String> autoList =
      List.of(
          "NONE",
          "SOURCE SIDE TWO PIECE",
          "SOURCE SIDE FULL WING AUTO",
          "URI AUTO",
          "URI + 1",
          "MIDLINE 5, 4, 3",
          "MIDLINE 5, 4",
          "MIDLINE 4, 5",
          "MIDLINE 4, 3",
          "AMP SIDE TWO PIECE",
          "AMP SIDE THREE PIECE",
          "AMP SIDE FOUR PIECE",
          "AMP SIDE FULL WING AUTO",
          "CENTER TWO PIECE",
          "CENTER-AMP FULL WING AUTO",
          "CENTER-SOURCE FULL WING AUTO",
          "ROBOTEER'S COMPLIMENT",
          "ROBOTEER'S COMPLIMENT - 1",
          "GTFOOTW",
          "LEAVE");

  // TODO: FIX OPTIONAL RUNTIME ERROR
}

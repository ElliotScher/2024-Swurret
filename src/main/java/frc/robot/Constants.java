// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.FieldConstants.StagingLocations;
import frc.robot.FieldConstants.Subwoofer;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final RobotType ROBOT = RobotType.ROBOT_SIM;

  public static Mode getMode() {
    switch (ROBOT) {
      case SNAPBACK:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    SNAPBACK,
    ROBOT_2K24_TEST,
    ROBOT_SIM
  }

  public static class AutoPathPoints {
    public static final double ROBOT_Y_LENGTH = Units.inchesToMeters(34.6875);
    public static final double ROBOT_X_LENGTH = Units.inchesToMeters(34.875);

    // Robot transforms
    public static final Transform2d ROBOT_FRONT_RIGHT_CORNER_TO_CENTER =
        new Transform2d(-ROBOT_X_LENGTH / 2, ROBOT_Y_LENGTH / 2, Rotation2d.fromDegrees(0.0));
    public static final Transform2d ROBOT_FRONT_LEFT_CORNER_TO_CENTER =
        new Transform2d(-ROBOT_X_LENGTH / 2, -ROBOT_Y_LENGTH / 2, Rotation2d.fromDegrees(0.0));
    public static final Transform2d ROBOT_FRONT_TO_CENTER =
        new Transform2d(-ROBOT_X_LENGTH / 2, 0.0, Rotation2d.fromDegrees(0.0));

    // Starting poses
    public static final Pose2d SOURCE_SIDE_SUBWOOFER =
        Subwoofer.sourceFaceCorner.transformBy(ROBOT_FRONT_RIGHT_CORNER_TO_CENTER);
    public static final Pose2d AMP_SIDE_SUBWOOFER =
        Subwoofer.ampFaceCorner.transformBy(ROBOT_FRONT_LEFT_CORNER_TO_CENTER);
    public static final Pose2d CENTER_SUBWOOFER =
        Subwoofer.centerFace.transformBy(ROBOT_FRONT_TO_CENTER);
    public static final Pose2d OPPONENT_SOURCE_AGAINST_ALLIANCE_WALL =
        new Pose2d(0.39, 1.95, Rotation2d.fromDegrees(180.0));
    public static final Pose2d OPPONENT_SOURCE_AGAINST_STARTING_LINE =
        new Pose2d(1.43, 1.58, Rotation2d.fromDegrees(180.0));

    // Note poses
    public static final Pose2d NOTE_1 =
        new Pose2d(
            StagingLocations.spikeX - 1,
            StagingLocations.spikeFirstY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_2 =
        new Pose2d(
            StagingLocations.spikeX - 1,
            StagingLocations.spikeFirstY + StagingLocations.spikeSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_3 =
        new Pose2d(
            StagingLocations.spikeX - 1,
            StagingLocations.spikeFirstY + 2 * StagingLocations.spikeSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_4 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_5 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_6 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + 2 * StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_7 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + 3 * StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_8 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + 4 * StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));

    // Shot poses
    public static final Pose2d SOURCE_SIDE_SHOT =
        new Pose2d(3.20, 3.20, Rotation2d.fromDegrees(148.47));
    public static final Pose2d SOURCE_SIDE_PODIUM_SHOT =
        new Pose2d(1.9, 4.57, Rotation2d.fromDegrees(144.46));
    public static final Pose2d CENTER_SHOT = new Pose2d(2.11, 5.52, Rotation2d.fromDegrees(180));
    public static final Pose2d CENTER_SHOT_FAR = new Pose2d(3, 5.52, Rotation2d.fromDegrees(180));
    public static final Pose2d AMP_SIDE_SHOT = new Pose2d(2.10, 6.54, new Rotation2d(-153.43));

    // Misc poses
    public static final Pose2d OUT_OF_THE_WAY =
        new Pose2d(3.05, 0.39, Rotation2d.fromDegrees(180.0));
    public static final Pose2d SUBWOOFER_CENTER_DROPPED_NOTE =
        new Pose2d(1.35, 5.53, Rotation2d.fromDegrees(0.0));
  }

  /** Checks whether the robot the correct mode is selected when deploying. */
  public static void main(String... args) {
    if (ROBOT == RobotType.ROBOT_SIM) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}

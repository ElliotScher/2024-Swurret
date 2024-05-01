package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public final class DriveConstants {
  public static final double WHEEL_RADIUS;
  public static final double TRACK_WIDTH_X;
  public static final double TRACK_WIDTH_Y;
  public static final double MAX_LINEAR_VELOCITY;
  public static final double MAX_ANGULAR_VELOCITY;
  public static final double DRIVE_BASE_RADIUS;
  public static final SwerveDriveKinematics KINEMATICS;
  public static final String CANIVORE;
  public static final int PIGEON_2_DEVICE_ID;

  static {
    WHEEL_RADIUS = Units.inchesToMeters(2.0);
    TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    MAX_LINEAR_VELOCITY = Units.feetToMeters(17.5);
    DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
    MAX_ANGULAR_VELOCITY = MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS;
    KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d[] {
              new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
              new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
              new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
              new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
            });
    CANIVORE = "drive";
    PIGEON_2_DEVICE_ID = 1;
    switch (Constants.ROBOT) {
      case SNAPBACK:
      case ROBOT_2K24_TEST:
      case ROBOT_SIM:
        break;
    }
  }
}

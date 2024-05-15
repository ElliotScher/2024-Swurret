package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
  public static final LoggedTunableNumber KP = new LoggedTunableNumber("Hood/Kp");
  public static final LoggedTunableNumber KD = new LoggedTunableNumber("Hood/Kd");
  public static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Hood/Max Velocity");
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Hood/Max Acceleration");
  public static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Hood/Stowed Position");
  public static final LoggedTunableNumber AMP_POSITION =
      new LoggedTunableNumber("Hood/Amp Position");
  public static final LoggedTunableNumber SOURCE_SIDE_FEED_POSITION =
      new LoggedTunableNumber("Hood/Source Side Feed Position");
  public static final LoggedTunableNumber AMP_SIDE_FEED_POSITION =
      new LoggedTunableNumber("Hood/Amp Side Feed Position");
  public static final LoggedTunableNumber MIN_POSITION =
      new LoggedTunableNumber("Hood/Minimum Angle");
  public static final LoggedTunableNumber MAX_POSITION =
      new LoggedTunableNumber("Hood/Maximum Angle");
  public static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Hood/Goal Tolerance");

  public static final int DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INTERTIA;
  public static final double ARM_LENGTH;
  public static final Rotation2d MINIMUM_ANGLE;
  public static final Rotation2d MAXIMUM_ANGLE;
  public static final boolean SIMULATE_GRAVITY;
  public static final Rotation2d STARTING_POSITION;
  public static final DCMotor MOTOR_CONFIG;

  static {
    GOAL_TOLERANCE.initDefault(0.017);

    DEVICE_ID = 7;
    SUPPLY_CURRENT_LIMIT = 60.0;
    GEAR_RATIO = 85.0;
    MOMENT_OF_INTERTIA = 0.004;
    ARM_LENGTH = Units.inchesToMeters(12.0);
    MINIMUM_ANGLE = Rotation2d.fromDegrees(0.0);
    MAXIMUM_ANGLE = Rotation2d.fromDegrees(45.0);
    SIMULATE_GRAVITY = true;
    STARTING_POSITION = Rotation2d.fromDegrees(38.0);
    MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        SOURCE_SIDE_FEED_POSITION.initDefault(0.3);
        AMP_SIDE_FEED_POSITION.initDefault(0.5);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(20.0);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KP.initDefault(90.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(1000.0);
        MAX_ACCELERATION.initDefault(1000.0);
        STOWED_POSITION.initDefault(Units.degreesToRadians(38.0));
        AMP_POSITION.initDefault(Units.degreesToRadians(15.0));
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
    }
  }
}

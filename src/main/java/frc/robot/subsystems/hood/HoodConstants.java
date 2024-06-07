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
  public static final LoggedTunableNumber MIN_POSITION =
      new LoggedTunableNumber("Hood/Minimum Angle");
  public static final LoggedTunableNumber MAX_POSITION =
      new LoggedTunableNumber("Hood/Maximum Angle");
  public static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Hood/Goal Tolerance");

  public static final int LEFT_DEVICE_ID;
  public static final int RIGHT_DEVICE_ID;

  public static final int CAN_TIMEOUT;
  public static final double NOMINAL_VOLTAGE;
  public static final int VELOCITY_MEASUREMENT_PERIOD;
  public static final int VELOCITY_AVERAGE_DEPTH;
  public static final int CURRENT_LIMIT;
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final double ARM_LENGTH;
  public static final Rotation2d MINIMUM_ANGLE;
  public static final Rotation2d MAXIMUM_ANGLE;
  public static final boolean SIMULATE_GRAVITY;
  public static final Rotation2d STARTING_POSITION;
  public static final DCMotor MOTOR_CONFIG;

  static {
    GOAL_TOLERANCE.initDefault(0.017);

    LEFT_DEVICE_ID = 14;
    RIGHT_DEVICE_ID = 15;
    CURRENT_LIMIT = 60;
    CAN_TIMEOUT = 250;
    NOMINAL_VOLTAGE = 12.0;
    VELOCITY_MEASUREMENT_PERIOD = 10;
    VELOCITY_AVERAGE_DEPTH = 2;
    ARM_LENGTH = Units.inchesToMeters(12.575608);
    MINIMUM_ANGLE = Rotation2d.fromDegrees(0.0);
    MAXIMUM_ANGLE = Rotation2d.fromDegrees(180.0);
    SIMULATE_GRAVITY = true;
    STARTING_POSITION = Rotation2d.fromDegrees(0.0);
    GEAR_RATIO = 220.0 / 18.0;
    MOMENT_OF_INERTIA = 0.004;
    switch (Constants.ROBOT) {
      case ROBOT_SPARK_FLEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_TALONFX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_NEO:
        MOTOR_CONFIG = DCMotor.getNEO(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_VORTEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_FALCON500:
        MOTOR_CONFIG = DCMotor.getFalcon500(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_FALCON500_FOC:
        MOTOR_CONFIG = DCMotor.getFalcon500Foc(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_KRAKEN_X60:
        MOTOR_CONFIG = DCMotor.getKrakenX60(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_KRAKEN_X60_FOC:
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      default:
        MOTOR_CONFIG = DCMotor.getNeoVortex(2);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
    }
  }
}

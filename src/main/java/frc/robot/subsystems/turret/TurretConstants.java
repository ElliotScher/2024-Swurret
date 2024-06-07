package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class TurretConstants {
  public static final LoggedTunableNumber KP = new LoggedTunableNumber("Turret/Kp");
  public static final LoggedTunableNumber KD = new LoggedTunableNumber("Turret/Kd");
  public static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Turret/Max Velocity");
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Turret/Max Acceleration");
  public static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Turret/Stowed Position");
  public static final LoggedTunableNumber FEED_POSITION =
      new LoggedTunableNumber("Turret/Feed Position");
  public static final LoggedTunableNumber AMP_POSITION =
      new LoggedTunableNumber("Turret/Amp Position");
  public static final LoggedTunableNumber MIN_POSITION =
      new LoggedTunableNumber("Turret/Minimum Angle");
  public static final LoggedTunableNumber MAX_POSITION =
      new LoggedTunableNumber("Turret/Maximum Angle");
  public static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Turret/Goal Tolerance");

  public static final int DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final Rotation2d MINIMUM_ANGLE;
  public static final Rotation2d MAXIMUM_ANGLE;
  public static final Rotation2d STARTING_POSITION;

  public static final int CURRENT_LIMIT;
  public static final int CAN_TIMEOUT;
  public static final double NOMINAL_VOLTAGE;
  public static final int VELOCITY_MEASUREMENT_PERIOD;
  public static final int VELOCITY_AVERAGE_DEPTH;

  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;

  static {
    GEAR_RATIO = 115.0 / 10.0;
    MOMENT_OF_INERTIA = 0.004;
    GOAL_TOLERANCE.initDefault(0.017);
    CURRENT_LIMIT = 40;
    CAN_TIMEOUT = 250;
    NOMINAL_VOLTAGE = 12.0;
    VELOCITY_MEASUREMENT_PERIOD = 10;
    VELOCITY_AVERAGE_DEPTH = 2;
    DEVICE_ID = 11;
    SUPPLY_CURRENT_LIMIT = 60.0;
    MINIMUM_ANGLE = Rotation2d.fromDegrees(-360.0);
    MAXIMUM_ANGLE = Rotation2d.fromDegrees(360.0);
    STARTING_POSITION = Rotation2d.fromDegrees(0.0);
    switch (Constants.ROBOT) {
      case ROBOT_SPARK_FLEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_TALONFX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_NEO:
        MOTOR_CONFIG = DCMotor.getNEO(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_VORTEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_FALCON500:
        MOTOR_CONFIG = DCMotor.getFalcon500(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_FALCON500_FOC:
        MOTOR_CONFIG = DCMotor.getFalcon500Foc(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_KRAKEN_X60:
        MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      case ROBOT_SIM_KRAKEN_X60_FOC:
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        break;
      default:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(25.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        FEED_POSITION.initDefault(0.3);
        AMP_POSITION.initDefault(0.0);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
    }
  }
}

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final LoggedTunableNumber RATIO = new LoggedTunableNumber("Shooter/Ratio");
  public static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Kp");
  public static final LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Kd");
  public static final LoggedTunableNumber KS_LEFT = new LoggedTunableNumber("Shooter/Left Ks");
  public static final LoggedTunableNumber KV_LEFT = new LoggedTunableNumber("Shooter/Left Kv");
  public static final LoggedTunableNumber KA_LEFT = new LoggedTunableNumber("Shooter/Left Ka");
  public static final LoggedTunableNumber KS_RIGHT = new LoggedTunableNumber("Shooter/Right Ks");
  public static final LoggedTunableNumber KV_RIGHT = new LoggedTunableNumber("Shooter/Right Kv");
  public static final LoggedTunableNumber KA_RIGHT = new LoggedTunableNumber("Shooter/Right Ka");
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Shooter/Max Acceleration");
  public static final LoggedTunableNumber IDLE_SPEED =
      new LoggedTunableNumber("Shooter/Default Speed");
  public static final LoggedTunableNumber AMP_SPEED = new LoggedTunableNumber("Shooter/Amp Speed");
  public static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Shooter/Goal Tolerance");

  public static final int LEFT_DEVICE_ID;
  public static final int RIGHT_DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double ENABLED_SPIN_DISTANCE;
  public static final double SPEAKER_TO_ROBOT_SPIN_THRESHOLD;
  public static final double SYSID_DELAY;
  public static final double RAMP_RATE_VOLTAGE;
  public static final double RAMP_RATE_SECONDS;
  public static final double STEP_VOLTAGE;
  public static final double SYSID_TIMEOUT;
  public static final double WHEEL_DIAMETER;
  public static final double FLOOR_TO_HOOD_PIVOT;

  public static final int CAN_TIMEOUT;
  public static final double NOMINAL_VOLTAGE;
  public static final int VELOCITY_MEASUREMENT_PERIOD;
  public static final int VELOCITY_AVERAGE_DEPTH;
  public static final int CURRENT_LIMIT;

  static {
    LEFT_DEVICE_ID = 16;
    RIGHT_DEVICE_ID = 17;
    SUPPLY_CURRENT_LIMIT = 60.0;
    ENABLED_SPIN_DISTANCE = 1.5;
    SPEAKER_TO_ROBOT_SPIN_THRESHOLD = 15;
    SYSID_DELAY = 4.0;
    RAMP_RATE_VOLTAGE = 0.2;
    RAMP_RATE_SECONDS = 1.0;
    STEP_VOLTAGE = 3.5;
    SYSID_TIMEOUT = 10.0;
    GEAR_RATIO = 1.0;
    MOMENT_OF_INERTIA = 0.004;
    CURRENT_LIMIT = 60;
    CAN_TIMEOUT = 250;
    NOMINAL_VOLTAGE = 12.0;
    VELOCITY_MEASUREMENT_PERIOD = 10;
    VELOCITY_AVERAGE_DEPTH = 2;
    WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    FLOOR_TO_HOOD_PIVOT = 0.33625;
    switch (Constants.ROBOT) {
      case ROBOT_SPARK_FLEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));
        break;
      case ROBOT_TALONFX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      case ROBOT_SIM_NEO:
        MOTOR_CONFIG = DCMotor.getNEO(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      case ROBOT_SIM_VORTEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      case ROBOT_SIM_FALCON500:
        MOTOR_CONFIG = DCMotor.getFalcon500(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      case ROBOT_SIM_FALCON500_FOC:
        MOTOR_CONFIG = DCMotor.getFalcon500Foc(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      case ROBOT_SIM_KRAKEN_X60:
        MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      case ROBOT_SIM_KRAKEN_X60_FOC:
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        IDLE_SPEED.initDefault(600);
        GOAL_TOLERANCE.initDefault(10);
        MAX_ACCELERATION.initDefault(425);
        KS_LEFT.initDefault(0.13053);
        KV_LEFT.initDefault(0.0072202);
        KA_LEFT.initDefault(0.0010296);
        KS_RIGHT.initDefault(0.15054);
        KV_RIGHT.initDefault(0.0068511);
        KA_RIGHT.initDefault(0.0011501);
        AMP_SPEED.initDefault(Units.rotationsPerMinuteToRadiansPerSecond(100));

        break;
      default:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
    }
  }
}

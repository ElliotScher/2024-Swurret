package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final int LEFT_DEVICE_ID;
  public static final int RIGHT_DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final double ENABLED_SPIN_DISTANCE;
  public static final double SPEAKER_TO_ROBOT_SPIN_THRESHOLD;
  public static final double SYSID_DELAY;
  public static final double RAMP_RATE_VOLTAGE;
  public static final double RAMP_RATE_SECONDS;
  public static final double STEP_VOLTAGE;
  public static final double SYSID_TIMEOUT;
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
  public static final LoggedTunableNumber DEFAULT_SPEED =
      new LoggedTunableNumber("Shooter/Default Speed");
  public static final LoggedTunableNumber SOURCE_FEED_SPEED =
      new LoggedTunableNumber("Shooter/Source Feed Speed");
  public static final LoggedTunableNumber AMP_FEED_SPEED =
      new LoggedTunableNumber("Shooter/Amp Feed Speed");
  public static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Shooter/Goal Tolerance");

  static {
    LEFT_DEVICE_ID = 16;
    RIGHT_DEVICE_ID = 10;
    SUPPLY_CURRENT_LIMIT = 60.0;
    GEAR_RATIO = 68.0 / 24.0;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    ENABLED_SPIN_DISTANCE = 1.5;
    SPEAKER_TO_ROBOT_SPIN_THRESHOLD = 15;
    SYSID_DELAY = 4.0;
    RAMP_RATE_VOLTAGE = 0.2;
    RAMP_RATE_SECONDS = 1.0;
    STEP_VOLTAGE = 3.5;
    SYSID_TIMEOUT = 10.0;
    GOAL_TOLERANCE.initDefault(10);
    MAX_ACCELERATION.initDefault(425);
    KS_LEFT.initDefault(0.13053);
    KV_LEFT.initDefault(0.0072202);
    KA_LEFT.initDefault(0.0010296);
    KS_RIGHT.initDefault(0.15054);
    KV_RIGHT.initDefault(0.0068511);
    KA_RIGHT.initDefault(0.0011501);
    SOURCE_FEED_SPEED.initDefault(550);
    AMP_FEED_SPEED.initDefault(400);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        DEFAULT_SPEED.initDefault(600);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(400);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(600);
        break;
      default:
        break;
    }
  }
}

package frc.robot.subsystems.serializer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class SerializerConstants {
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;

  public static final int DEVICE_ID;
  public static final int CURRENT_LIMIT;
  public static final int CAN_TIMEOUT;
  public static final double NOMINAL_VOLTAGE;
  public static final int VELOCITY_MEASUREMENT_PERIOD;
  public static final int VELOCITY_AVERAGE_DEPTH;

  static {
    GEAR_RATIO = 1.0;
    MOMENT_OF_INERTIA = 0.004;
    DEVICE_ID = 10;
    CURRENT_LIMIT = 40;
    CAN_TIMEOUT = 250;
    NOMINAL_VOLTAGE = 12.0;
    VELOCITY_MEASUREMENT_PERIOD = 10;
    VELOCITY_AVERAGE_DEPTH = 2;
    switch (Constants.ROBOT) {
      case ROBOT_SPARK_FLEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        break;
      case ROBOT_TALONFX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        break;
      case ROBOT_SIM_NEO:
        MOTOR_CONFIG = DCMotor.getNEO(1);
        break;
      case ROBOT_SIM_VORTEX:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
        break;
      case ROBOT_SIM_FALCON500:
        MOTOR_CONFIG = DCMotor.getFalcon500(1);
        break;
      case ROBOT_SIM_FALCON500_FOC:
        MOTOR_CONFIG = DCMotor.getFalcon500Foc(1);
        break;
      case ROBOT_SIM_KRAKEN_X60:
        MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        break;
      case ROBOT_SIM_KRAKEN_X60_FOC:
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        break;
      default:
        MOTOR_CONFIG = DCMotor.getNeoVortex(1);
    }
  }
}

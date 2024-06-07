package frc.robot.util;

import frc.robot.Constants;

public class ChoreoUtil {
  public static final double MOTOR_MAX_TORQUE;
  public static final double MOTOR_MAX_VELOCITY;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_SPARK_FLEX:
        MOTOR_MAX_TORQUE = 1.0236966824644549;
        MOTOR_MAX_VELOCITY = 5427.200000000001;
        break;
      case ROBOT_TALONFX:
        MOTOR_MAX_TORQUE = 1.162295081967213;
        MOTOR_MAX_VELOCITY = 4800.0;
        break;
      case ROBOT_SIM_NEO:
        MOTOR_MAX_TORQUE = 1.087292817679558;
        MOTOR_MAX_VELOCITY = 4704.0;
        break;
      case ROBOT_SIM_VORTEX:
        MOTOR_MAX_TORQUE = 1.0236966824644549;
        MOTOR_MAX_VELOCITY = 5427.200000000001;
        break;
      case ROBOT_SIM_FALCON500:
        MOTOR_MAX_TORQUE = 1.0949416342412452;
        MOTOR_MAX_VELOCITY = 5104.0;
        break;
      case ROBOT_SIM_FALCON500_FOC:
        MOTOR_MAX_TORQUE = 1.1526315789473685;
        MOTOR_MAX_VELOCITY = 4864.0;
        break;
      case ROBOT_SIM_KRAKEN_X60:
        MOTOR_MAX_TORQUE = 1.162295081967213;
        MOTOR_MAX_VELOCITY = 4800.0;
        break;
      case ROBOT_SIM_KRAKEN_X60_FOC:
        MOTOR_MAX_TORQUE = 1.163975155279503;
        MOTOR_MAX_VELOCITY = 4640.0;
        break;
      default:
        MOTOR_MAX_TORQUE = 1.0236966824644549;
        MOTOR_MAX_VELOCITY = 5427.200000000001;
        break;
    }
  }
}

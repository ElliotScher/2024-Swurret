package frc.robot.subsystems.serializer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class SerializerConstants {
  public static final LoggedTunableNumber SHOOT_VOLTAGE =
      new LoggedTunableNumber("Serializer/Shoot Voltage");
  public static final LoggedTunableNumber INTAKE_VOLTAGE =
      new LoggedTunableNumber("Serializer/Intake Voltage");

  public static final int DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final int SENSOR_CHANNEL;

  static {
    SHOOT_VOLTAGE.initDefault(12.0);
    INTAKE_VOLTAGE.initDefault(12.0);

    SUPPLY_CURRENT_LIMIT = 40.0;
    GEAR_RATIO = 2.0;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    SENSOR_CHANNEL = 0;
    switch (Constants.ROBOT) {
      case SNAPBACK:
        DEVICE_ID = 3;
        break;
      case ROBOT_2K24_TEST:
        DEVICE_ID = 48;
        break;
      case ROBOT_SIM:
      default:
        DEVICE_ID = -1;
    }
  }
}

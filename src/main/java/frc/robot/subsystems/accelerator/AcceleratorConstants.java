package frc.robot.subsystems.accelerator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class AcceleratorConstants {
  public static final LoggedTunableNumber SHOOT_VOLTAGE =
      new LoggedTunableNumber("Accelerator/Shoot Voltage");

  public static final int DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;

  static {
    SHOOT_VOLTAGE.initDefault(12.0);

    DEVICE_ID = 15;
    SUPPLY_CURRENT_LIMIT = 40.0;
    GEAR_RATIO = 2.0;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    switch (Constants.ROBOT) {
      case SNAPBACK:
      case ROBOT_2K24_TEST:
      case ROBOT_SIM:
        break;
    }
  }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final int SOLENOID_CHANNEL;
  public static final int DEVICE_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final DCMotor MOTOR_CONFIG;
  public static final double GEAR_RATIO;
  public static final double MOMENT_OF_INERTIA;
  public static final LoggedTunableNumber ROLLERS_VOLTAGE =
      new LoggedTunableNumber("Intake/Voltage");

  static {
    SOLENOID_CHANNEL = 7;
    DEVICE_ID = 17;
    SUPPLY_CURRENT_LIMIT = 60.0;
    GEAR_RATIO = 1.6;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    ROLLERS_VOLTAGE.initDefault(12);
    switch (Constants.ROBOT) {
      case SNAPBACK:
      case ROBOT_2K24_TEST:
      case ROBOT_SIM:
        break;
    }
  }
}

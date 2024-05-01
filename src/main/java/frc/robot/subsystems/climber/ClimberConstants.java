package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimberConstants {
  public static final int LEFT_CLIMBER_TALON_DEVICE_ID;
  public static final int RIGHT_CLIMBER_TALON_DEVICE_ID;
  public static final int SOLENOID_CHANNEL;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double GEAR_RATIO;
  public static final double DRUM_CIRCUMFERENCE;
  public static final DCMotor MOTOR_CONFIG;
  public static final double CARRIAGE_MASS;
  public static final double MINIMUM_HEIGHT;
  public static final double MAXIMUM_HEIGHT;
  public static final double STARTING_HEIGHT;
  public static final boolean SIMULATE_GRAVITY;
  public static final double LOCKING_DELAY;
  public static final double CLIMB_STAGE_1_VOLTAGE;
  public static final double CLIMB_TRANSITION_CURRENT_THRESHOLD;
  public static final double CLIMB_STAGE_2_VOLTAGE;
  public static final double CLIMB_STOP_POSITION_THRESHOLD;
  ;
  public static final double CLIMB_HOLD_VOLTAGE;
  public static final double CLIMB_ZERO_VOLTAGE;
  public static final double CLIMB_ZERO_CURRENT_THRESHOLD;
  public static final double CLIMB_POSITION_TOLERANCE;
  public static final LoggedTunableNumber KP = new LoggedTunableNumber("Climber/Kp");
  public static final LoggedTunableNumber KD = new LoggedTunableNumber("Climber/Kd");
  public static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Climber/Max Velocity");
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Climber/Max Acceleration");
  public static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Climber/Stowed Position");
  public static final LoggedTunableNumber CLIMB_POSITION =
      new LoggedTunableNumber("Climber/Climb Position");

  static {
    LEFT_CLIMBER_TALON_DEVICE_ID = 4;
    RIGHT_CLIMBER_TALON_DEVICE_ID = 5;
    SOLENOID_CHANNEL = 6;
    SUPPLY_CURRENT_LIMIT = 60.0;
    GEAR_RATIO = (58.0 / 16.0) * (64.0 / 18.0);
    CLIMB_POSITION.initDefault(12.5);
    DRUM_CIRCUMFERENCE = Units.inchesToMeters(2.64595) * Math.PI;
    MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    CARRIAGE_MASS = 7.0;
    MINIMUM_HEIGHT = 0.0;
    MAXIMUM_HEIGHT = 1.0;
    STARTING_HEIGHT = 0.0;
    SIMULATE_GRAVITY = false;
    LOCKING_DELAY = 0.25;
    CLIMB_STAGE_1_VOLTAGE = -6.0;
    CLIMB_TRANSITION_CURRENT_THRESHOLD = 25.0;
    CLIMB_STAGE_2_VOLTAGE = -8.0;
    CLIMB_STOP_POSITION_THRESHOLD = 0.5;
    CLIMB_HOLD_VOLTAGE = -1.1;
    CLIMB_ZERO_VOLTAGE = -1.0;
    CLIMB_ZERO_CURRENT_THRESHOLD = 2.0;
    CLIMB_POSITION_TOLERANCE = 1.0;
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(5);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(30);
        MAX_ACCELERATION.initDefault(20);
        STOWED_POSITION.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KP.initDefault(1.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(1.0);
        MAX_ACCELERATION.initDefault(1.0);
        STOWED_POSITION.initDefault(0.0);
        break;
    }
  }
}

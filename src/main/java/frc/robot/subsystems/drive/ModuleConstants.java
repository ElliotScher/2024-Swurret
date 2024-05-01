package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Builder;

public class ModuleConstants {
  public static final ModuleConfig frontLeftConfig;
  public static final ModuleConfig frontRightConfig;
  public static final ModuleConfig rearLeftConfig;
  public static final ModuleConfig rearRightConfig;
  public static final double ODOMETRY_FREQUENCY;
  public static final double OUT_OF_SYNC_THRESHOLD;
  public static final double DRIVE_GEAR_RATIO;
  public static final double TURN_GEAR_RATIO;
  public static final double DRIVE_CURRENT_LIMIT;
  public static final double TURN_CURRENT_LIMIT;
  public static final double DRIVE_MOMENT_OF_INERTIA;
  public static final double TURN_MOMENT_OF_INERTIA;
  public static final DCMotor DRIVE_MOTOR_CONFIG;
  public static final DCMotor TURN_MOTOR_CONFIG;
  public static final LoggedTunableNumber WHEEL_RADIUS =
      new LoggedTunableNumber("Drive/Wheel Radius");
  public static final LoggedTunableNumber DRIVE_KS = new LoggedTunableNumber("Drive/Drive Ks");
  public static final LoggedTunableNumber DRIVE_KV = new LoggedTunableNumber("Drive/Drive Kv");
  public static final LoggedTunableNumber DRIVE_KP = new LoggedTunableNumber("Drive/Drive Kp");
  public static final LoggedTunableNumber DRIVE_KD = new LoggedTunableNumber("Drive/Drive Kd");
  public static final LoggedTunableNumber TURN_KP = new LoggedTunableNumber("Drive/Turn Kp");
  public static final LoggedTunableNumber TURN_KD = new LoggedTunableNumber("Drive/Turn Kd");

  static {
    ODOMETRY_FREQUENCY = 250.0;
    OUT_OF_SYNC_THRESHOLD = Units.degreesToRadians(30.0);
    DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    TURN_GEAR_RATIO = 150.0 / 7.0;
    DRIVE_CURRENT_LIMIT = 40.0;
    TURN_CURRENT_LIMIT = 30.0;
    DRIVE_MOMENT_OF_INERTIA = 0.025;
    TURN_MOMENT_OF_INERTIA = 0.004;
    DRIVE_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    TURN_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KS.initDefault(0.063566);
        DRIVE_KV.initDefault(0.11799);
        DRIVE_KP.initDefault(0.13);
        DRIVE_KD.initDefault(0.0);
        TURN_KP.initDefault(9.0);
        TURN_KD.initDefault(0.0);
        frontLeftConfig =
            new ModuleConfig(
                new TalonFX(11, DriveConstants.CANIVORE),
                new TalonFX(12, DriveConstants.CANIVORE),
                new CANcoder(20, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(2.2304080655857224));
        frontRightConfig =
            new ModuleConfig(
                new TalonFX(8, DriveConstants.CANIVORE),
                new TalonFX(9, DriveConstants.CANIVORE),
                new CANcoder(21, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(-1.4910293258248433));
        rearLeftConfig =
            new ModuleConfig(
                new TalonFX(18, DriveConstants.CANIVORE),
                new TalonFX(19, DriveConstants.CANIVORE),
                new CANcoder(22, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(-0.2132233295161041));
        rearRightConfig =
            new ModuleConfig(
                new TalonFX(0, DriveConstants.CANIVORE),
                new TalonFX(1, DriveConstants.CANIVORE),
                new CANcoder(23, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(-1.4327380558851888));
        break;
      case ROBOT_2K24_TEST:
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KS.initDefault(0.14589);
        DRIVE_KV.initDefault(0.11156);
        DRIVE_KP.initDefault(0.13);
        DRIVE_KD.initDefault(0.0);
        TURN_KP.initDefault(9.0);
        TURN_KD.initDefault(0.0);
        frontLeftConfig =
            new ModuleConfig(
                new TalonFX(0, DriveConstants.CANIVORE),
                new TalonFX(1, DriveConstants.CANIVORE),
                new CANcoder(2, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(1.8775924843720249));
        frontRightConfig =
            new ModuleConfig(
                new TalonFX(10, DriveConstants.CANIVORE),
                new TalonFX(11, DriveConstants.CANIVORE),
                new CANcoder(12, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(1.9865051203119053));
        rearLeftConfig =
            new ModuleConfig(
                new TalonFX(20, DriveConstants.CANIVORE),
                new TalonFX(21, DriveConstants.CANIVORE),
                new CANcoder(22, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(1.1612234564294304));
        rearRightConfig =
            new ModuleConfig(
                new TalonFX(30, DriveConstants.CANIVORE),
                new TalonFX(31, DriveConstants.CANIVORE),
                new CANcoder(32, DriveConstants.CANIVORE),
                Rotation2d.fromRadians(-3.06182565261974));
        break;
      case ROBOT_SIM:
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KS.initDefault(-0.0081157);
        DRIVE_KV.initDefault(0.12821);
        DRIVE_KP.initDefault(0.039024);
        DRIVE_KD.initDefault(0.0);
        TURN_KP.initDefault(10.0);
        TURN_KD.initDefault(0.0);
        frontLeftConfig = null;
        frontRightConfig = null;
        rearLeftConfig = null;
        rearRightConfig = null;
        break;
      default:
        frontLeftConfig = null;
        frontRightConfig = null;
        rearLeftConfig = null;
        rearRightConfig = null;
        break;
    }
  }

  @Builder
  public record ModuleConfig(
      TalonFX drive, TalonFX turn, CANcoder cancoder, Rotation2d absoluteEncoderOffset) {}
}

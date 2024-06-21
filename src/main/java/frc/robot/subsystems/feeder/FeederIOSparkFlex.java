package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class FeederIOSparkFlex implements FeederIO {
  private final CANSparkFlex leaderSparkFlex;
  private final CANSparkFlex followerSparkFlex;

  private final RelativeEncoder leaderRelativeEncoder;
  private final RelativeEncoder followerRelativeEncoder;

  private final DigitalInput sensor;

  public FeederIOSparkFlex() {
    leaderSparkFlex = new CANSparkFlex(FeederConstants.LEADER_DEVICE_ID, MotorType.kBrushless);
    followerSparkFlex = new CANSparkFlex(FeederConstants.FOLLOWER_DEVICE_ID, MotorType.kBrushless);

    leaderRelativeEncoder = leaderSparkFlex.getEncoder();
    followerRelativeEncoder = followerSparkFlex.getEncoder();

    leaderSparkFlex.restoreFactoryDefaults();
    followerSparkFlex.restoreFactoryDefaults();

    leaderSparkFlex.setCANTimeout(FeederConstants.CAN_TIMEOUT);
    followerSparkFlex.setCANTimeout(FeederConstants.CAN_TIMEOUT);

    leaderSparkFlex.setSmartCurrentLimit(FeederConstants.CURRENT_LIMIT);
    followerSparkFlex.setSmartCurrentLimit(FeederConstants.CURRENT_LIMIT);

    leaderSparkFlex.enableVoltageCompensation(FeederConstants.NOMINAL_VOLTAGE);
    followerSparkFlex.enableVoltageCompensation(FeederConstants.NOMINAL_VOLTAGE);

    leaderRelativeEncoder.setMeasurementPeriod(FeederConstants.VELOCITY_MEASUREMENT_PERIOD);
    followerRelativeEncoder.setMeasurementPeriod(FeederConstants.VELOCITY_MEASUREMENT_PERIOD);

    leaderRelativeEncoder.setAverageDepth(FeederConstants.VELOCITY_AVERAGE_DEPTH);
    followerRelativeEncoder.setAverageDepth(FeederConstants.VELOCITY_AVERAGE_DEPTH);

    leaderSparkFlex.setCANTimeout(0);
    followerSparkFlex.setCANTimeout(0);

    followerSparkFlex.follow(leaderSparkFlex);

    leaderSparkFlex.burnFlash();
    followerSparkFlex.burnFlash();

    sensor = new DigitalInput(FeederConstants.SENSOR_CHANNEL);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.leaderPosition =
        Rotation2d.fromRotations(leaderRelativeEncoder.getPosition() / FeederConstants.GEAR_RATIO);
    inputs.leaderVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leaderRelativeEncoder.getVelocity() / FeederConstants.GEAR_RATIO);
    inputs.leaderAppliedVolts =
        leaderSparkFlex.getAppliedOutput() * leaderSparkFlex.getBusVoltage();
    inputs.leaderCurrentAmps = leaderSparkFlex.getOutputCurrent();
    inputs.leaderTempCelcius = leaderSparkFlex.getMotorTemperature();

    inputs.hasNote = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    leaderSparkFlex.setVoltage(volts);
  }
}

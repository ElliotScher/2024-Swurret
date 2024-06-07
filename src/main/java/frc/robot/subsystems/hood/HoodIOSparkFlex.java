package frc.robot.subsystems.hood;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class HoodIOSparkFlex implements HoodIO {
  private final CANSparkFlex leaderSparkFlex;
  private final CANSparkFlex followerSparkFlex;

  private final RelativeEncoder leftRelativeEncoder;
  private final RelativeEncoder rightRelativeEncoder;

  public HoodIOSparkFlex() {
    leaderSparkFlex = new CANSparkFlex(HoodConstants.LEFT_DEVICE_ID, MotorType.kBrushless);
    followerSparkFlex = new CANSparkFlex(HoodConstants.RIGHT_DEVICE_ID, MotorType.kBrushless);

    leftRelativeEncoder = leaderSparkFlex.getEncoder();
    rightRelativeEncoder = followerSparkFlex.getEncoder();

    leaderSparkFlex.restoreFactoryDefaults();
    followerSparkFlex.restoreFactoryDefaults();

    leaderSparkFlex.setCANTimeout(HoodConstants.CAN_TIMEOUT);
    followerSparkFlex.setCANTimeout(HoodConstants.CAN_TIMEOUT);

    leaderSparkFlex.setSmartCurrentLimit(HoodConstants.CURRENT_LIMIT);
    followerSparkFlex.setSmartCurrentLimit(HoodConstants.CURRENT_LIMIT);

    leaderSparkFlex.enableVoltageCompensation(HoodConstants.NOMINAL_VOLTAGE);
    followerSparkFlex.enableVoltageCompensation(HoodConstants.NOMINAL_VOLTAGE);

    leftRelativeEncoder.setMeasurementPeriod(HoodConstants.VELOCITY_MEASUREMENT_PERIOD);
    rightRelativeEncoder.setMeasurementPeriod(HoodConstants.VELOCITY_MEASUREMENT_PERIOD);

    leftRelativeEncoder.setAverageDepth(HoodConstants.VELOCITY_AVERAGE_DEPTH);
    rightRelativeEncoder.setAverageDepth(HoodConstants.VELOCITY_AVERAGE_DEPTH);

    leaderSparkFlex.setCANTimeout(0);
    followerSparkFlex.setCANTimeout(0);

    followerSparkFlex.follow(leaderSparkFlex, true);

    leaderSparkFlex.burnFlash();
    followerSparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position =
        Rotation2d.fromRotations(leftRelativeEncoder.getPosition() / HoodConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leftRelativeEncoder.getVelocity() / HoodConstants.GEAR_RATIO);
    inputs.appliedVolts = leaderSparkFlex.getAppliedOutput() * leaderSparkFlex.getBusVoltage();
    inputs.currentAmps = leaderSparkFlex.getOutputCurrent();
    inputs.tempCelcius = leaderSparkFlex.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    leaderSparkFlex.setVoltage(volts);
  }
}

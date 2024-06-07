package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkFlex implements ShooterIO {
  private final CANSparkFlex leftSparkFlex;
  private final CANSparkFlex rightSparkFlex;

  private final RelativeEncoder leftRelativeEncoder;
  private final RelativeEncoder rightRelativeEncoder;

  public ShooterIOSparkFlex() {
    leftSparkFlex = new CANSparkFlex(ShooterConstants.LEFT_DEVICE_ID, MotorType.kBrushless);
    rightSparkFlex = new CANSparkFlex(ShooterConstants.RIGHT_DEVICE_ID, MotorType.kBrushless);

    leftRelativeEncoder = leftSparkFlex.getEncoder();
    rightRelativeEncoder = rightSparkFlex.getEncoder();

    leftSparkFlex.restoreFactoryDefaults();
    rightSparkFlex.restoreFactoryDefaults();

    leftSparkFlex.setCANTimeout(ShooterConstants.CAN_TIMEOUT);
    rightSparkFlex.setCANTimeout(ShooterConstants.CAN_TIMEOUT);

    leftSparkFlex.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
    rightSparkFlex.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

    leftSparkFlex.enableVoltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);
    rightSparkFlex.enableVoltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);

    leftRelativeEncoder.setMeasurementPeriod(ShooterConstants.VELOCITY_MEASUREMENT_PERIOD);
    rightRelativeEncoder.setMeasurementPeriod(ShooterConstants.VELOCITY_MEASUREMENT_PERIOD);

    leftRelativeEncoder.setAverageDepth(ShooterConstants.VELOCITY_AVERAGE_DEPTH);
    rightRelativeEncoder.setAverageDepth(ShooterConstants.VELOCITY_AVERAGE_DEPTH);

    leftSparkFlex.setCANTimeout(0);
    rightSparkFlex.setCANTimeout(0);

    rightSparkFlex.follow(leftSparkFlex, true);

    leftSparkFlex.burnFlash();
    rightSparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftPosition =
        Rotation2d.fromRotations(leftRelativeEncoder.getPosition() / ShooterConstants.GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leftRelativeEncoder.getVelocity() / ShooterConstants.GEAR_RATIO);
    inputs.leftAppliedVolts = leftSparkFlex.getAppliedOutput() * leftSparkFlex.getBusVoltage();
    inputs.leftCurrentAmps = leftSparkFlex.getOutputCurrent();
    inputs.leftTempCelcius = leftSparkFlex.getMotorTemperature();

    inputs.rightPosition =
        Rotation2d.fromRotations(rightRelativeEncoder.getPosition() / ShooterConstants.GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            rightRelativeEncoder.getVelocity() / ShooterConstants.GEAR_RATIO);
    inputs.rightAppliedVolts = rightSparkFlex.getAppliedOutput() * rightSparkFlex.getBusVoltage();
    inputs.rightCurrentAmps = rightSparkFlex.getOutputCurrent();
    inputs.rightTempCelcius = rightSparkFlex.getMotorTemperature();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftSparkFlex.setVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightSparkFlex.setVoltage(volts);
  }
}

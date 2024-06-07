package frc.robot.subsystems.serializer;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SerializerIOSparkFlex implements SerializerIO {
  private final CANSparkFlex sparkFlex;
  private final RelativeEncoder relativeEncoder;

  public SerializerIOSparkFlex() {
    sparkFlex = new CANSparkFlex(SerializerConstants.DEVICE_ID, MotorType.kBrushless);
    relativeEncoder = sparkFlex.getEncoder();
    sparkFlex.restoreFactoryDefaults();
    sparkFlex.setCANTimeout(SerializerConstants.CAN_TIMEOUT);
    sparkFlex.setSmartCurrentLimit(SerializerConstants.CURRENT_LIMIT);
    sparkFlex.enableVoltageCompensation(SerializerConstants.NOMINAL_VOLTAGE);
    relativeEncoder.setMeasurementPeriod(SerializerConstants.VELOCITY_MEASUREMENT_PERIOD);
    relativeEncoder.setAverageDepth(SerializerConstants.VELOCITY_AVERAGE_DEPTH);
    sparkFlex.setCANTimeout(0);
    sparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    inputs.position =
        Rotation2d.fromRotations(relativeEncoder.getPosition() / SerializerConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            relativeEncoder.getVelocity() / SerializerConstants.GEAR_RATIO);
    inputs.appliedVolts = sparkFlex.getAppliedOutput() * sparkFlex.getBusVoltage();
    inputs.currentAmps = sparkFlex.getOutputCurrent();
    inputs.tempCelcius = sparkFlex.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    sparkFlex.setVoltage(volts);
  }
}

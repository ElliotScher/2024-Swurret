package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkFlex implements IntakeIO {
  private final CANSparkFlex sparkFlex;
  private final RelativeEncoder relativeEncoder;

  public IntakeIOSparkFlex() {
    sparkFlex = new CANSparkFlex(IntakeConstants.DEVICE_ID, MotorType.kBrushless);
    relativeEncoder = sparkFlex.getEncoder();
    sparkFlex.restoreFactoryDefaults();
    sparkFlex.setCANTimeout(IntakeConstants.CAN_TIMEOUT);
    sparkFlex.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    sparkFlex.enableVoltageCompensation(IntakeConstants.NOMINAL_VOLTAGE);
    relativeEncoder.setMeasurementPeriod(IntakeConstants.VELOCITY_MEASUREMENT_PERIOD);
    relativeEncoder.setAverageDepth(IntakeConstants.VELOCITY_AVERAGE_DEPTH);
    sparkFlex.setCANTimeout(0);
    sparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position =
        Rotation2d.fromRotations(relativeEncoder.getPosition() / IntakeConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            relativeEncoder.getVelocity() / IntakeConstants.GEAR_RATIO);
    inputs.appliedVolts = sparkFlex.getAppliedOutput() * sparkFlex.getBusVoltage();
    inputs.currentAmps = sparkFlex.getOutputCurrent();
    inputs.tempCelcius = sparkFlex.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    sparkFlex.setVoltage(volts);
  }
}

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TurretIOSparkFlex implements TurretIO {
  private final CANSparkFlex sparkFlex;
  private final RelativeEncoder relativeEncoder;

  public TurretIOSparkFlex() {
    sparkFlex = new CANSparkFlex(TurretConstants.DEVICE_ID, MotorType.kBrushless);
    relativeEncoder = sparkFlex.getEncoder();
    sparkFlex.restoreFactoryDefaults();
    sparkFlex.setCANTimeout(TurretConstants.CAN_TIMEOUT);
    sparkFlex.setSmartCurrentLimit(TurretConstants.CURRENT_LIMIT);
    sparkFlex.enableVoltageCompensation(TurretConstants.NOMINAL_VOLTAGE);
    relativeEncoder.setMeasurementPeriod(TurretConstants.VELOCITY_MEASUREMENT_PERIOD);
    relativeEncoder.setAverageDepth(TurretConstants.VELOCITY_AVERAGE_DEPTH);
    sparkFlex.setCANTimeout(0);
    sparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.position =
        Rotation2d.fromRotations(relativeEncoder.getPosition() / TurretConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            relativeEncoder.getVelocity() / TurretConstants.GEAR_RATIO);
    inputs.appliedVolts = sparkFlex.getAppliedOutput() * sparkFlex.getBusVoltage();
    inputs.currentAmps = sparkFlex.getOutputCurrent();
    inputs.tempCelcius = sparkFlex.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    sparkFlex.setVoltage(volts);
  }
}

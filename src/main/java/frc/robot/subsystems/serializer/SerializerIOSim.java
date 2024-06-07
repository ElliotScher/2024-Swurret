package frc.robot.subsystems.serializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SerializerIOSim implements SerializerIO {
  private DCMotorSim motorSim =
      new DCMotorSim(
          SerializerConstants.MOTOR_CONFIG,
          SerializerConstants.GEAR_RATIO,
          SerializerConstants.MOMENT_OF_INERTIA);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position =
        Rotation2d.fromRadians(motorSim.getAngularPositionRad() / SerializerConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        motorSim.getAngularVelocityRadPerSec() / SerializerConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.tempCelcius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }
}

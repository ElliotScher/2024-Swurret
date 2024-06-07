package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
  private DCMotorSim motorSim =
      new DCMotorSim(
          HoodConstants.MOTOR_CONFIG, HoodConstants.GEAR_RATIO, HoodConstants.MOMENT_OF_INERTIA);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position =
        Rotation2d.fromRadians(motorSim.getAngularPositionRad() / HoodConstants.GEAR_RATIO);
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec() / HoodConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.tempCelcius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }
}

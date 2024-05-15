package frc.robot.subsystems.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class KickerIOSim implements KickerIO {
  private DCMotorSim motorSim =
      new DCMotorSim(
          KickerConstants.MOTOR_CONFIG,
          KickerConstants.GEAR_RATIO,
          KickerConstants.MOMENT_OF_INERTIA);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position = Rotation2d.fromRadians(motorSim.getAngularPositionRad());
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(motorSim.getCurrentDrawAmps())};
    inputs.tempCelcius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }
}

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  private DCMotorSim motorSim =
      new DCMotorSim(
          FeederConstants.MOTOR_CONFIG,
          FeederConstants.GEAR_RATIO,
          FeederConstants.MOMENT_OF_INERTIA);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.leaderPosition =
        Rotation2d.fromRadians(motorSim.getAngularPositionRad() / FeederConstants.GEAR_RATIO);
    inputs.leaderVelocityRadPerSec =
        motorSim.getAngularVelocityRadPerSec() / FeederConstants.GEAR_RATIO;
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.leaderTempCelcius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }
}

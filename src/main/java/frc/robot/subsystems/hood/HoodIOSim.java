package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
  private SingleJointedArmSim motorSim =
      new SingleJointedArmSim(
          HoodConstants.MOTOR_CONFIG,
          HoodConstants.GEAR_RATIO,
          HoodConstants.GEAR_RATIO,
          HoodConstants.ARM_LENGTH,
          HoodConstants.MINIMUM_ANGLE.getRadians(),
          HoodConstants.MAXIMUM_ANGLE.getRadians(),
          HoodConstants.SIMULATE_GRAVITY,
          HoodConstants.STARTING_POSITION.getRadians());

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position = Rotation2d.fromRadians(motorSim.getAngleRads());
    inputs.velocityRadPerSec = motorSim.getVelocityRadPerSec();
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

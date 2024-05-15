package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          HoodConstants.MOTOR_CONFIG,
          HoodConstants.GEAR_RATIO,
          HoodConstants.MOMENT_OF_INTERTIA,
          HoodConstants.ARM_LENGTH,
          HoodConstants.MINIMUM_ANGLE.getRadians(),
          HoodConstants.MAXIMUM_ANGLE.getRadians(),
          HoodConstants.SIMULATE_GRAVITY,
          HoodConstants.STARTING_POSITION.getRadians());
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    armSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    inputs.tempCelcius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    armSim.setInputVoltage(appliedVolts);
  }
}

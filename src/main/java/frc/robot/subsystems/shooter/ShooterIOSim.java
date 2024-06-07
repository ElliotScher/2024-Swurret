package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim leftMotorSim =
      new DCMotorSim(
          ShooterConstants.MOTOR_CONFIG,
          ShooterConstants.GEAR_RATIO,
          ShooterConstants.MOMENT_OF_INERTIA);
  private DCMotorSim rightMotorSim =
      new DCMotorSim(
          ShooterConstants.MOTOR_CONFIG,
          ShooterConstants.GEAR_RATIO,
          ShooterConstants.MOMENT_OF_INERTIA);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    leftMotorSim.update(Constants.LOOP_PERIOD_SECS);
    rightMotorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.leftPosition =
        Rotation2d.fromRadians(leftMotorSim.getAngularPositionRad() / ShooterConstants.GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        leftMotorSim.getAngularVelocityRadPerSec() / ShooterConstants.GEAR_RATIO;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = leftMotorSim.getCurrentDrawAmps();
    inputs.leftTempCelcius = 0.0;

    inputs.rightPosition =
        Rotation2d.fromRadians(rightMotorSim.getAngularPositionRad() / ShooterConstants.GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        rightMotorSim.getAngularVelocityRadPerSec() / ShooterConstants.GEAR_RATIO;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = rightMotorSim.getCurrentDrawAmps();
    inputs.rightTempCelcius = 0.0;
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftMotorSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightMotorSim.setInputVoltage(rightAppliedVolts);
  }
}

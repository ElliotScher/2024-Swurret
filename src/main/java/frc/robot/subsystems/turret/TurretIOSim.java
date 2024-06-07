package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO {
  private DCMotorSim motorSim =
      new DCMotorSim(
          TurretConstants.MOTOR_CONFIG,
          TurretConstants.GEAR_RATIO,
          TurretConstants.MOMENT_OF_INERTIA);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position =
        Rotation2d.fromRadians(motorSim.getAngularPositionRad() / TurretConstants.GEAR_RATIO);
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec() / TurretConstants.GEAR_RATIO;
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

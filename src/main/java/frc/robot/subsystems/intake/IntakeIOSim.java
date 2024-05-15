package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim motorSim =
      new DCMotorSim(
          IntakeConstants.MOTOR_CONFIG,
          IntakeConstants.GEAR_RATIO,
          IntakeConstants.MOMENT_OF_INERTIA);

  private double rollersAppliedVolts = 0.0;
  private boolean leftPosition = false;
  private boolean rightPosition = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.rollersPosition = Rotation2d.fromRadians(motorSim.getAngularPositionRad());
    inputs.rollersVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.rollersAppliedVolts = rollersAppliedVolts;
    inputs.rollersCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.rollersTempCelcius = 0.0;

    inputs.leftPosition = leftPosition;
    inputs.rightPosition = rightPosition;
  }

  @Override
  public void setRollersVoltage(double volts) {
    rollersAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(rollersAppliedVolts);
  }

  @Override
  public void setIntakePosition(boolean position) {
    leftPosition = position;
    rightPosition = position;
  }
}

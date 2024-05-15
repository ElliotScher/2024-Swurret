package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
  private ElevatorSim leftClimberSim =
      new ElevatorSim(
          ClimberConstants.MOTOR_CONFIG,
          ClimberConstants.GEAR_RATIO,
          ClimberConstants.CARRIAGE_MASS,
          ClimberConstants.DRUM_CIRCUMFERENCE,
          ClimberConstants.MINIMUM_HEIGHT,
          ClimberConstants.MAXIMUM_HEIGHT,
          ClimberConstants.SIMULATE_GRAVITY,
          ClimberConstants.STARTING_HEIGHT);

  private ElevatorSim rightClimberSim =
      new ElevatorSim(
          ClimberConstants.MOTOR_CONFIG,
          ClimberConstants.GEAR_RATIO,
          ClimberConstants.CARRIAGE_MASS,
          ClimberConstants.DRUM_CIRCUMFERENCE,
          ClimberConstants.MINIMUM_HEIGHT,
          ClimberConstants.MAXIMUM_HEIGHT,
          ClimberConstants.SIMULATE_GRAVITY,
          ClimberConstants.STARTING_HEIGHT);

  private double leftAppliedVolts;
  private double rightAppliedVolts;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    leftClimberSim.update(Constants.LOOP_PERIOD_SECS);
    rightClimberSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.leftPositionMeters = leftClimberSim.getPositionMeters();
    inputs.leftVelocityMetersPerSec = leftClimberSim.getVelocityMetersPerSecond();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = leftClimberSim.getCurrentDrawAmps();
    inputs.leftTempCelcius = 0.0;

    inputs.rightPositionMeters = rightClimberSim.getPositionMeters();
    inputs.rightVelocityMetersPerSec = rightClimberSim.getVelocityMetersPerSecond();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = rightClimberSim.getCurrentDrawAmps();
    inputs.rightTempCelcius = 0.0;
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftClimberSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightClimberSim.setInputVoltage(rightAppliedVolts);
  }
}

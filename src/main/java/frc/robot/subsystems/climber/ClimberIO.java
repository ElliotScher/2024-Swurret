package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double leftPositionMeters = 0.0;
    public double leftVelocityMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTempCelcius = 0.0;

    public double rightPositionMeters = 0.0;
    public double rightVelocityMetersPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTempCelcius = 0.0;

    public boolean lockedPosition = true;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setLeftVoltage(double volts) {}

  public default void setRightVoltage(double volts) {}

  public default void setLock(boolean isLocked) {}

  public default void resetPosition() {}
}

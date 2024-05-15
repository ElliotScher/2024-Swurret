package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d rollersPosition = new Rotation2d();
    public double rollersVelocityRadPerSec = 0.0;
    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double rollersTempCelcius = 0.0;

    public boolean leftPosition = false;
    public boolean rightPosition = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollersVoltage(double volts) {}

  public default void setIntakePosition(boolean isDeployed) {}
}

package frc.robot.subsystems.serializer;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SerializerIO {
  @AutoLog
  public static class SerializerIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelcius = 0.0;
  }

  public default void updateInputs(SerializerIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}

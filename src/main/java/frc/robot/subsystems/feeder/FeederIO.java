package frc.robot.subsystems.feeder;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public Rotation2d leaderPosition = new Rotation2d();
    public double leaderVelocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double leaderTempCelcius = 0.0;

    public Rotation2d followerPosition = new Rotation2d();
    public double followerVelocityRadPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerTempCelcius = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default boolean getSensorReading() {
    return false;
  }
}

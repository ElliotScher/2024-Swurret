package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Rotation2d tx = new Rotation2d();
    public Rotation2d ty = new Rotation2d();
    public boolean tv = false;
    public double megaTag1Timestamp = 0.0;
    public Pose3d megaTag1RobotPose = new Pose3d();
    public double megaTag2Timestamp = 0.0;
    public Pose3d megaTag2RobotPose = new Pose3d();
    public long pipeline = 0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void enableLEDs() {}

  public default void disableLEDs() {}

  public default void setPipeline(double pipeline) {}
}

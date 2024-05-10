package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public Rotation2d xOffset = new Rotation2d();
    public Rotation2d yOffset = new Rotation2d();
    public boolean targetAquired = false;
    public double primaryPoseTimestamp = 0.0;
    public Pose3d primaryPose = new Pose3d();
    public double secondaryPoseTimestamp = 0.0;
    public Pose3d secondaryPose = new Pose3d();
    public long pipeline = 0;
    public CameraType cameraType = CameraType.GENERIC_PHOTON_VISION_CAMERA;
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default void enableLEDs() {}

  public default void disableLEDs() {}

  public default void setPipeline(double pipeline) {}
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Camera {
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

  private final CameraIO io;
  private final int index;

  public Camera(CameraIO io, int index) {
    this.io = io;
    this.index = index;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Vision/Camera" + Integer.toString(index), inputs);
  }

  public Rotation2d getTx() {
    return inputs.xOffset;
  }

  public Rotation2d getTy() {
    return inputs.yOffset;
  }

  public boolean getTv() {
    return inputs.targetAquired;
  }

  public double getMegaTag1Timestamp() {
    return inputs.primaryPoseTimestamp;
  }

  public Pose3d getMegaTag1RobotPose() {
    return inputs.primaryPose;
  }

  public double getMegaTag2Timestamp() {
    return inputs.secondaryPoseTimestamp;
  }

  public Pose3d getMegaTag2RobotPose() {
    return inputs.secondaryPose;
  }

  public long getPipeline() {
    return inputs.pipeline;
  }

  public CameraType getCameraType() {
    return inputs.cameraType;
  }

  public void setPipeline(int pipeline) {
    io.setPipeline(pipeline);
  }

  public void enableLEDs() {
    io.enableLEDs();
  }

  public void disableLEDs() {
    io.disableLEDs();
  }
}

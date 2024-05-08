package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Camera {
  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
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
    return inputs.tx;
  }

  public Rotation2d getTy() {
    return inputs.ty;
  }

  public boolean getTv() {
    return inputs.tv;
  }

  public double getMegaTag1Timestamp() {
    return inputs.megaTag1Timestamp;
  }

  public Pose3d getMegaTag1RobotPose() {
    return inputs.megaTag1RobotPose;
  }

  public double getMegaTag2Timestamp() {
    return inputs.megaTag2Timestamp;
  }

  public Pose3d getMegaTag2RobotPose() {
    return inputs.megaTag2RobotPose;
  }

  public long getPipeline() {
    return inputs.pipeline;
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

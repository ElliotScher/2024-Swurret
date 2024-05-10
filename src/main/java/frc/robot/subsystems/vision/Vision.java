package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.VirtualSubsystem;

public class Vision extends VirtualSubsystem {
  private final Camera[] cameras = new Camera[4]; // FL, FR, BL, BR

  public Vision(CameraIO flIO, CameraIO frIO, CameraIO blIO, CameraIO brIO) {
    cameras[0] = new Camera(flIO, 0);
    cameras[1] = new Camera(frIO, 1);
    cameras[2] = new Camera(blIO, 2);
    cameras[3] = new Camera(brIO, 3);
  }

  @Override
  public void periodic() {}

  public boolean getValidTarget() {
    boolean isValidTarget = true;
    for (Camera camera : cameras) {
      isValidTarget = camera.getTv();
    }

    return isValidTarget;
  }

  public Pose3d[] getPrimaryVisionPoses() {
    Pose3d[] poseArray = new Pose3d[4];
    for (int i = 0; i < cameras.length; i++) {
      poseArray[i] = cameras[i].getMegaTag1RobotPose();
    }

    return poseArray;
  }

  public Pose3d[] getSecondaryVisionPoses() {
    Pose3d[] poseArray = new Pose3d[4];
    for (int i = 0; i < cameras.length; i++) {
      poseArray[i] = cameras[i].getMegaTag2RobotPose();
    }

    return poseArray;
  }

  public double[] getPrimaryPoseTimestamps() {
    double[] timestampArray = new double[4];
    for (int i = 0; i < cameras.length; i++) {
      timestampArray[i] = cameras[i].getMegaTag1Timestamp();
    }

    return timestampArray;
  }

  public double[] getSecondaryPoseTimestamps() {
    double[] timestampArray = new double[4];
    for (int i = 0; i < cameras.length; i++) {
      timestampArray[i] = cameras[i].getMegaTag2Timestamp();
    }

    return timestampArray;
  }

  public CameraType[] getCameraTypes() {
    CameraType[] cameraTypeArray = new CameraType[4];
    for (int i = 0; i < cameraTypeArray.length; i++) {
      cameraTypeArray[i] = cameras[i].getCameraType();
    }
    return cameraTypeArray;
  }

  public Command enableLEDs() {
    return Commands.runOnce(
        () -> {
          for (Camera camera : cameras) {
            camera.enableLEDs();
          }
        });
  }

  public Command disableLEDs() {
    return Commands.runOnce(
        () -> {
          for (Camera camera : cameras) {
            camera.disableLEDs();
          }
        });
  }

  public Command blinkLEDs() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                camera.enableLEDs();
              }
            }),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                camera.disableLEDs();
              }
            }),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                camera.enableLEDs();
              }
            }),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                camera.disableLEDs();
              }
            }),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                camera.enableLEDs();
              }
            }),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                camera.disableLEDs();
              }
            }),
        Commands.waitSeconds(VisionConstants.BLINK_TIME));
  }
}

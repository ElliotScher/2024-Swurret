package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {
  private final String name;
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs;

  public Vision(String name, VisionIO io) {
    this.name = name;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public boolean getTv() {
    return inputs.tv;
  }

  public Rotation2d getTx() {
    return inputs.tx;
  }

  public Rotation2d getTy() {
    return inputs.ty;
  }

  public Pose3d getMegaTag1Pose() {
    return inputs.megaTag1RobotPose;
  }

  public Pose3d getMegaTag2Pose() {
    return inputs.megaTag2RobotPose;
  }

  public double getMegaTag1Timestamp() {
    return inputs.megaTag1Timestamp;
  }

  public double getMegaTag2Timestamp() {
    return inputs.megaTag2Timestamp;
  }

  public void disableLEDs() {
    io.disableLEDs();
  }

  public Command setPipeline(VisionPipeline noteTrackingPipeline) {
    return Commands.runOnce(() -> io.setPipeline(noteTrackingPipeline.pipeline));
  }

  public Command blinkLEDs() {
    return Commands.sequence(
        Commands.runOnce(() -> io.enableLEDs()),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(() -> io.disableLEDs()),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(() -> io.enableLEDs()),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(() -> io.disableLEDs()),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(() -> io.enableLEDs()),
        Commands.waitSeconds(VisionConstants.BLINK_TIME),
        Commands.runOnce(() -> io.disableLEDs()),
        Commands.waitSeconds(VisionConstants.BLINK_TIME));
  }
}

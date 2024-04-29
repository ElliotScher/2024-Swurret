package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {

  private static final double BLINK_TIME = 0.067;

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

  public Pose3d getRobotPose() {
    return inputs.robotPose;
  }

  public double getTimestamp() {
    return inputs.timeStamp;
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
        Commands.waitSeconds(BLINK_TIME),
        Commands.runOnce(() -> io.disableLEDs()),
        Commands.waitSeconds(BLINK_TIME),
        Commands.runOnce(() -> io.enableLEDs()),
        Commands.waitSeconds(BLINK_TIME),
        Commands.runOnce(() -> io.disableLEDs()),
        Commands.waitSeconds(BLINK_TIME),
        Commands.runOnce(() -> io.enableLEDs()),
        Commands.waitSeconds(BLINK_TIME),
        Commands.runOnce(() -> io.disableLEDs()),
        Commands.waitSeconds(BLINK_TIME));
  }
}

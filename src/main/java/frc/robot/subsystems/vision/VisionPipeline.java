package frc.robot.subsystems.vision;

public enum VisionPipeline {
  Center(0),
  AmpSide(1),
  SourceSide(2);

  public final int pipeline;

  private VisionPipeline(int pipeline) {
    this.pipeline = pipeline;
  }
}

package frc.robot.subsystems.vision;

public enum VisionNoteTrackingPipeline {
  Center(0),
  AmpSide(1),
  SourceSide(2);

  public final int pipeline;

  private VisionNoteTrackingPipeline(int pipeline) {
    this.pipeline = pipeline;
  }
}

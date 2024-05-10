package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FieldConstants;

public class CameraIOLimelight3 implements CameraIO {
  private final NetworkTable table;
  private final DoubleSubscriber tx;
  private final DoubleSubscriber ty;
  private final DoubleSubscriber tv;
  private final DoubleArraySubscriber megaTag1;
  private final DoubleArraySubscriber megaTag2;
  private final IntegerSubscriber pipeline;

  public CameraIOLimelight3(int index) {
    table = NetworkTableInstance.getDefault().getTable("limelight-" + index);
    tx = table.getDoubleTopic("tx").subscribe(0.0);
    ty = table.getDoubleTopic("ty").subscribe(0.0);
    tv = table.getDoubleTopic("tv").subscribe(0.0);
    megaTag1 =
        table
            .getDoubleArrayTopic("botpose")
            .subscribe(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    megaTag2 =
        table
            .getDoubleArrayTopic("botpose_orb")
            .subscribe(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    pipeline = table.getIntegerTopic("pipeline").subscribe(0);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    inputs.primaryPoseTimestamp =
        megaTag1.getLastChange() * 0.000001
            - megaTag1.get()[6]
                * 0.001; // Calculate the time (in seconds) when the Limelight captured the frame

    inputs.secondaryPoseTimestamp =
        megaTag2.getLastChange() * 0.000001
            - megaTag2.get()[6]
                * 0.001; // Calculate the time (in seconds) when the Limelight captured the frame

    inputs.xOffset = Rotation2d.fromDegrees(tx.get());
    inputs.yOffset = Rotation2d.fromDegrees(ty.get());
    inputs.targetAquired = tv.get() != 0;
    inputs.primaryPose =
        new Pose3d(
            megaTag1.get()[0] + FieldConstants.fieldLength / 2.0,
            megaTag1.get()[1] + FieldConstants.fieldWidth / 2.0,
            megaTag1.get()[2],
            new Rotation3d(
                Units.degreesToRadians(megaTag1.get()[3]),
                Units.degreesToRadians(megaTag1.get()[4]),
                Units.degreesToRadians(megaTag1.get()[5])));
    inputs.secondaryPose =
        new Pose3d(
            megaTag2.get()[0] + FieldConstants.fieldLength / 2.0,
            megaTag2.get()[1] + FieldConstants.fieldWidth / 2.0,
            megaTag2.get()[2],
            new Rotation3d(
                Units.degreesToRadians(megaTag2.get()[3]),
                Units.degreesToRadians(megaTag2.get()[4]),
                Units.degreesToRadians(megaTag2.get()[5])));
    inputs.pipeline = pipeline.get();
    inputs.cameraType = CameraType.LIMELIGHT_3;
  }

  @Override
  public void enableLEDs() {
    table.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void disableLEDs() {
    table.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void setPipeline(double pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }
}

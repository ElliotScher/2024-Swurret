package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

public class CameraIOSim implements CameraIO {
  private final Supplier<Pose2d> poseSupplier;

  private final Transform3d cameraTransform;
  private final Pose3d[] targetPoses;

  public CameraIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    cameraTransform = new Transform3d(0.3, 0, 0.1, new Rotation3d(0, -Math.PI / 4.0, 0));
    targetPoses =
        new Pose3d[] {
          FieldConstants.aprilTags.getTagPose(4).get(), FieldConstants.aprilTags.getTagPose(7).get()
        };
  }

  public void updateInputs(CameraIOInputs inputs) {
    Pose3d cameraPose = new Pose3d(poseSupplier.get()).transformBy(cameraTransform);
    Double closestNorm = null;
    Rotation2d tx = null;
    Rotation2d ty = null;
    Pose3d targetRelativePose;

    for (int i = 0; i < targetPoses.length; i++) {
      targetRelativePose = targetPoses[i].relativeTo(cameraPose);
      Double currentNorm = targetRelativePose.getTranslation().getNorm();

      double xyAngle = Math.atan(targetRelativePose.getY() / targetRelativePose.getX());
      double xzAngle = Math.atan(targetRelativePose.getZ() / targetRelativePose.getX());

      boolean isValid =
          (Math.abs(xyAngle) < CameraConstants.LIMELIGHT_HORIZONTAL_FOV.getRadians() / 2.0)
              && (Math.abs(xzAngle) < CameraConstants.LIMELIGHT_VERTICAL_FOV.getRadians() / 2.0);

      if (isValid && (closestNorm == null || currentNorm < closestNorm)) {
        tx = new Rotation2d(-xyAngle);
        ty = new Rotation2d(xzAngle);
        closestNorm = currentNorm;
      }
    }

    inputs.megaTag1Timestamp = Timer.getFPGATimestamp();
    inputs.megaTag2Timestamp = Timer.getFPGATimestamp();
    inputs.tv = closestNorm != null;
    if (inputs.tv) {
      inputs.tx = tx;
      inputs.ty = ty;
      inputs.megaTag1RobotPose = new Pose3d(poseSupplier.get());
      inputs.megaTag2RobotPose = new Pose3d(poseSupplier.get());
    }
    inputs.pipeline = 0;
  }
}

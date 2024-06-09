package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Mechanism3d {
  private Mechanism3d() {}

  public static final Pose3d[] getPoses(Rotation2d turretAngle, Rotation2d hoodAngle) {
    Pose3d turretPose =
        new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, turretAngle.getRadians()));
    Pose3d hoodPose =
        new Pose3d(
            0.0,
            0.0,
            0.2515 + 0.085,
            new Rotation3d(hoodAngle.getRadians(), 0.0, turretAngle.getRadians()));

    return new Pose3d[] {turretPose, hoodPose};
  }
}

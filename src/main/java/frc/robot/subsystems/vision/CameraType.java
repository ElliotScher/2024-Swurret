package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.GenericPhotonVisionCameraConstants;
import frc.robot.subsystems.vision.VisionConstants.Limelight3Constants;
import frc.robot.subsystems.vision.VisionConstants.Limelight3GConstants;

public enum CameraType {
  LIMELIGHT_3(
      Limelight3Constants.MEGA_TAG_1_STANDARD_DEVIATIONS,
      Limelight3Constants.MEGA_TAG_2_STANDARD_DEVIATIONS),
  LIMELIGHT_3G(
      Limelight3GConstants.MEGA_TAG_1_STANDARD_DEVIATIONS,
      Limelight3GConstants.MEGA_TAG_2_STANDARD_DEVIATIONS),
  GENERIC_PHOTON_VISION_CAMERA(GenericPhotonVisionCameraConstants.STANDARD_DEVIATIONS);

  public final Matrix<N3, N1> primaryStandardDeviations;
  public final Matrix<N3, N1> secondaryStandardDeviations;

  private CameraType(
      Matrix<N3, N1> primaryStandardDeviations, Matrix<N3, N1> secondaryStandardDeviations) {
    this.primaryStandardDeviations = primaryStandardDeviations;
    this.secondaryStandardDeviations = secondaryStandardDeviations;
  }

  private CameraType(Matrix<N3, N1> standardDeviations) {
    this.primaryStandardDeviations = standardDeviations;
    this.secondaryStandardDeviations = null;
  }
}

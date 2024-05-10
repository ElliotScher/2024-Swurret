package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
  public static final double BLINK_TIME = 0.067;

  public static final Matrix<N3, N1> DEFAULT_STANDARD_DEVIATIONS = VecBuilder.fill(0.0, 0.0, 0.0);

  public class Limelight3Constants {
    public static final Matrix<N3, N1> MEGA_TAG_1_STANDARD_DEVIATIONS =
        VecBuilder.fill(0.0, 0.0, 0.0);
    public static final Matrix<N3, N1> MEGA_TAG_2_STANDARD_DEVIATIONS =
        VecBuilder.fill(0.0, 0.0, 0.0);
  }

  public class Limelight3GConstants {
    public static final Matrix<N3, N1> MEGA_TAG_1_STANDARD_DEVIATIONS =
        VecBuilder.fill(0.0, 0.0, 0.0);
    public static final Matrix<N3, N1> MEGA_TAG_2_STANDARD_DEVIATIONS =
        VecBuilder.fill(0.0, 0.0, 0.0);
  }

  public class GenericPhotonVisionCameraConstants {
    public static final Matrix<N3, N1> STANDARD_DEVIATIONS = VecBuilder.fill(0.0, 0.0, 0.0);
  }
}

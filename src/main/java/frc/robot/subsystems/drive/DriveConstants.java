package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Builder;

public final class DriveConstants {

  public static final DriveConfig driveConfig =
      switch (Constants.ROBOT) {
        case SNAPBACK, ROBOT_2K24_TEST, ROBOT_SIM -> DriveConfig.builder()
            .wheelRadius(Units.inchesToMeters(2.0))
            .trackWidthX(Units.inchesToMeters(25.0))
            .trackWidthY(Units.inchesToMeters(25.0))
            .maxLinearVelocity(17.5)
            .build();
        default -> throw new IllegalArgumentException("Unexpected value: " + Constants.getMode());
      };

  @Builder
  public record DriveConfig(
      double wheelRadius, double trackWidthX, double trackWidthY, double maxLinearVelocity) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }

    public double maxAngularVelocity() {
      return maxLinearVelocity / driveBaseRadius();
    }
  }
}

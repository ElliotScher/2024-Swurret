package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.physics.SimulationManager;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.vision.Camera;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingDoubleTreeMap speakerShotSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShotSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap speakerShotAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShotAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  @Getter
  private static ControlData controlData =
      new ControlData(
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          0.0,
          new Rotation2d(),
          0.0,
          new Rotation2d());

  @Getter @Setter private static double speakerFlywheelCompensation = 0.0;
  @Getter @Setter private static double speakerAngleCompensation = 0.0;

  private static SwerveDrivePoseEstimator poseEstimator;

  private static Supplier<Rotation2d> robotHeadingSupplier;
  private static DoubleSupplier robotYawVelocitySupplier;
  private static Supplier<Translation2d> robotFieldRelativeVelocitySupplier;
  private static Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private static Supplier<Camera[]> camerasSupplier;
  private static BooleanSupplier targetAquiredSupplier;
  private static Supplier<Optional<Pose3d>[]> visionPrimaryPosesSupplier;
  private static Supplier<Optional<Pose3d>[]> visionSecondaryPosesSupplier;
  private static Supplier<double[]> visionFrameTimestampSupplier;

  static {
    // Units: radians per second
    speakerShotSpeedMap.put(0.0, 1200.0);
    speakerShotSpeedMap.put(10.0, 1200.0);

    // Units: radians per second
    feedShotSpeedMap.put(0.0, 100.0);
    feedShotSpeedMap.put(10.0, 1000.0);

    // Units: radians
    speakerShotAngleMap.put(1.063443124193179, 0.5988516509065093);
    speakerShotAngleMap.put(1.5647293542039766, 0.7489817842632993);
    speakerShotAngleMap.put(2.3767267895164355, 0.9241623548399363);
    speakerShotAngleMap.put(2.619896271754951, 0.9992479449503855);
    speakerShotAngleMap.put(3.2534138460851483, 1.0593195065516758);
    speakerShotAngleMap.put(4.041779551869629, 1.124463298150421);
    speakerShotAngleMap.put(4.977461632381792, 1.1894826195309027);
    speakerShotAngleMap.put(5.791561476171746, 1.2245281547841378);
    speakerShotAngleMap.put(7.330916052912086, 1.274594190521463);
    speakerShotAngleMap.put(8.653328997969203, 1.2996275995450375);

    // Units: radians
    feedShotAngleMap.put(0.0, 0.0);
    feedShotAngleMap.put(10.0, 0.0);

    // Units: seconds
    timeOfFlightMap.put(1.1389823269997401, 0.1);
    timeOfFlightMap.put(2.5163366438541535, 0.14);
    timeOfFlightMap.put(3.174748321223645, 0.155);
    timeOfFlightMap.put(3.5880720870692264, 0.18);
    timeOfFlightMap.put(4.96042522950525, 0.21);
    timeOfFlightMap.put(5.973200881287628, 0.26);
  }

  public RobotState(
      Supplier<Rotation2d> robotHeadingSupplier,
      DoubleSupplier robotYawVelocitySupplier,
      Supplier<Translation2d> robotFieldRelativeVelocitySupplier,
      Supplier<SwerveModulePosition[]> modulePositionSupplier,
      Supplier<Camera[]> camerasSupplier,
      BooleanSupplier targetAquiredSupplier,
      Supplier<Optional<Pose3d>[]> visionPrimaryPosesSupplier,
      Supplier<Optional<Pose3d>[]> visionSecondaryPosesSupplier,
      Supplier<double[]> visionFrameTimestampSupplier) {
    RobotState.robotHeadingSupplier = robotHeadingSupplier;
    RobotState.robotYawVelocitySupplier = robotYawVelocitySupplier;
    RobotState.robotFieldRelativeVelocitySupplier = robotFieldRelativeVelocitySupplier;
    RobotState.modulePositionSupplier = modulePositionSupplier;
    RobotState.camerasSupplier = camerasSupplier;
    RobotState.targetAquiredSupplier = targetAquiredSupplier;
    RobotState.visionPrimaryPosesSupplier = visionPrimaryPosesSupplier;
    RobotState.visionSecondaryPosesSupplier = visionSecondaryPosesSupplier;
    RobotState.visionFrameTimestampSupplier = visionFrameTimestampSupplier;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            robotHeadingSupplier.get(),
            modulePositionSupplier.get(),
            new Pose2d(),
            DriveConstants.ODOMETRY_STANDARD_DEVIATIONS,
            VecBuilder.fill(0.0, 0.0, 0.0));
  }

  private static final void addVisionMeasurement() {
    if (targetAquiredSupplier.getAsBoolean()
        && robotYawVelocitySupplier.getAsDouble() < Units.degreesToRadians(720.0)) {
      for (int i = 0; i < visionPrimaryPosesSupplier.get().length; i++) {
        if (visionSecondaryPosesSupplier.get()[i].isPresent()) {
          double xyStddev =
              camerasSupplier.get()[i].getPrimaryXYStandardDeviationCoefficient()
                  * Math.pow(camerasSupplier.get()[i].getAverageDistance(), 2.0)
                  / camerasSupplier.get()[i].getTotalTargets()
                  * camerasSupplier.get()[i].getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              visionPrimaryPosesSupplier.get()[i].get().toPose2d(),
              visionFrameTimestampSupplier.get()[i],
              VecBuilder.fill(xyStddev, xyStddev, Double.POSITIVE_INFINITY));
        }
      }
      for (int i = 0; i < visionSecondaryPosesSupplier.get().length; i++) {
        if (visionSecondaryPosesSupplier.get()[i].isPresent()) {
          double xyStddev =
              camerasSupplier.get()[i].getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camerasSupplier.get()[i].getAverageDistance(), 2.0)
                  / camerasSupplier.get()[i].getTotalTargets()
                  * camerasSupplier.get()[i].getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              visionSecondaryPosesSupplier.get()[i].get().toPose2d(),
              visionFrameTimestampSupplier.get()[i],
              VecBuilder.fill(xyStddev, xyStddev, Double.POSITIVE_INFINITY));
        }
      }
    }
  }

  public static void periodic() {
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), robotHeadingSupplier.get(), modulePositionSupplier.get());
    addVisionMeasurement();

    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    Translation2d ampPose = AllianceFlipUtil.apply(FieldConstants.ampCenter);
    double distanceToSpeaker =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(speakerPose);
    double distanceToAmp =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(ampPose);
    Translation2d effectiveSpeakerAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(
                robotFieldRelativeVelocitySupplier
                    .get()
                    .times(timeOfFlightMap.get(distanceToSpeaker)));
    Translation2d effectiveFeedAmpAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(
                robotFieldRelativeVelocitySupplier.get().times(timeOfFlightMap.get(distanceToAmp)));
    double effectiveDistanceToSpeaker = effectiveSpeakerAimingPose.getDistance(speakerPose);
    double effectiveDistanceToAmp = effectiveFeedAmpAimingPose.getDistance(ampPose);

    Rotation2d speakerTurretAngle =
        speakerPose.minus(effectiveSpeakerAimingPose).getAngle().minus(robotHeadingSupplier.get());
    Rotation2d feedAmpTurretAngle =
        ampPose.minus(effectiveFeedAmpAimingPose).getAngle().minus(robotHeadingSupplier.get());
    controlData =
        new ControlData(
            speakerTurretAngle,
            feedAmpTurretAngle,
            feedAmpTurretAngle,
            speakerShotSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(speakerShotAngleMap.get(effectiveDistanceToSpeaker)),
            feedShotSpeedMap.get(effectiveDistanceToAmp),
            new Rotation2d(feedShotAngleMap.get(effectiveDistanceToAmp)));

    SimulationManager.periodic();

    Logger.recordOutput("RobotState/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/Effective Speaker Aiming Pose",
        new Pose2d(effectiveSpeakerAimingPose, new Rotation2d()));
    Logger.recordOutput(
        "RobotState/Effective Feed-Amp Aiming Pose",
        new Pose2d(effectiveFeedAmpAimingPose, new Rotation2d()));
    Logger.recordOutput("RobotState/Effective Distance To Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput("RobotState/Effective Distance To Amp", effectiveDistanceToAmp);
    Logger.recordOutput(
        "RobotState/ControlData/Speaker Turret Angle", controlData.speakerTurretAngle());
    Logger.recordOutput("RobotState/ControlData/Feed Turret Angle", controlData.feedTurretAngle());
    Logger.recordOutput("RobotState/ControlData/Amp Turret Angle", controlData.ampTurretAngle());
    Logger.recordOutput(
        "RobotState/ControlData/Speaker Shot Speed", controlData.speakerShotSpeed());
    Logger.recordOutput(
        "RobotState/ControlData/Speaker Hood Angle", controlData.speakerHoodAngle());
    Logger.recordOutput("RobotState/ControlData/Feed Shot Speed", controlData.feedShotSpeed());
    Logger.recordOutput("RobotState/ControlData/Feed Hood Angle", controlData.feedHoodAngle());
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(robotHeadingSupplier.get(), modulePositionSupplier.get(), pose);
  }

  public static record ControlData(
      Rotation2d speakerTurretAngle,
      Rotation2d feedTurretAngle,
      Rotation2d ampTurretAngle,
      double speakerShotSpeed,
      Rotation2d speakerHoodAngle,
      double feedShotSpeed,
      Rotation2d feedHoodAngle) {}
}

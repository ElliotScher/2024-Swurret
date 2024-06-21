package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.physics.SimulationManager;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.vision.CameraType;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.AllianceFlipUtil;
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

  private static final Alert secondaryPosesNullAlert =
      new Alert("SECONDARY VISION POSES ARE NULL", AlertType.INFO);

  @Getter
  private static StateCache stateCache =
      new StateCache(
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
  private static Supplier<Translation2d> robotFieldRelativeVelocitySupplier;
  private static Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private static Supplier<CameraType[]> camerasSupplier;
  private static Supplier<Pose3d[]> visionPrimaryPosesSupplier;
  private static Supplier<Pose3d[]> visionSecondaryPosesSupplier;
  private static Supplier<double[]> visionPrimaryPoseTimestampsSupplier;
  private static Supplier<double[]> visionSecondaryPoseTimestampsSupplier;

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
    timeOfFlightMap.put(0.0, 0.0);
  }

  public RobotState(
      Supplier<Rotation2d> robotHeadingSupplier,
      Supplier<Translation2d> robotFieldRelativeVelocitySupplier,
      Supplier<SwerveModulePosition[]> modulePositionSupplier,
      Supplier<CameraType[]> camerasSupplier,
      Supplier<Pose3d[]> visionPrimaryPosesSupplier,
      Supplier<Pose3d[]> visionSecondaryPosesSupplier,
      Supplier<double[]> visionPrimaryPoseTimestampsSupplier,
      Supplier<double[]> visionSecondaryPoseTimestampsSupplier) {
    RobotState.robotHeadingSupplier = robotHeadingSupplier;
    RobotState.robotFieldRelativeVelocitySupplier = robotFieldRelativeVelocitySupplier;
    RobotState.modulePositionSupplier = modulePositionSupplier;
    RobotState.camerasSupplier = camerasSupplier;
    RobotState.visionPrimaryPosesSupplier = visionPrimaryPosesSupplier;
    RobotState.visionSecondaryPosesSupplier = visionSecondaryPosesSupplier;
    RobotState.visionPrimaryPoseTimestampsSupplier = visionPrimaryPoseTimestampsSupplier;
    RobotState.visionSecondaryPoseTimestampsSupplier = visionSecondaryPoseTimestampsSupplier;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            robotHeadingSupplier.get(),
            modulePositionSupplier.get(),
            new Pose2d(),
            DriveConstants.ODOMETRY_STANDARD_DEVIATIONS,
            VisionConstants.DEFAULT_STANDARD_DEVIATIONS);
  }

  public static void periodic() {
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), robotHeadingSupplier.get(), modulePositionSupplier.get());
    for (int i = 0; i < visionPrimaryPosesSupplier.get().length; i++) {
      poseEstimator.addVisionMeasurement(
          visionPrimaryPosesSupplier.get()[i].toPose2d(),
          visionPrimaryPoseTimestampsSupplier.get()[i],
          camerasSupplier.get()[i].primaryStandardDeviations);
    }
    if (!secondaryPosesNullAlert.isActive()) {
      try {
        for (int i = 0; i < visionSecondaryPosesSupplier.get().length; i++) {
          poseEstimator.addVisionMeasurement(
              visionSecondaryPosesSupplier.get()[i].toPose2d(),
              visionSecondaryPoseTimestampsSupplier.get()[i],
              camerasSupplier.get()[i].secondaryStandardDeviations);
        }
        secondaryPosesNullAlert.set(false);
      } catch (Exception e) {
        secondaryPosesNullAlert.set(true);
      }
    }

    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    Translation2d ampPose = AllianceFlipUtil.apply(FieldConstants.ampCenter);
    double distanceToSpeaker =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(speakerPose);
    Translation2d effectiveAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(
                robotFieldRelativeVelocitySupplier
                    .get()
                    .times(timeOfFlightMap.get(distanceToSpeaker)));
    double effectiveDistanceToSpeaker = effectiveAimingPose.getDistance(speakerPose);
    double effectiveDistanceToAmp = effectiveAimingPose.getDistance(ampPose);

    Rotation2d speakerTurretAngle =
        speakerPose.minus(effectiveAimingPose).getAngle().minus(robotHeadingSupplier.get());
    Rotation2d feedAmpTurretAngle =
        ampPose.minus(effectiveAimingPose).getAngle().minus(robotHeadingSupplier.get());
    stateCache =
        new StateCache(
            speakerTurretAngle,
            feedAmpTurretAngle,
            feedAmpTurretAngle,
            speakerShotSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(speakerShotAngleMap.get(effectiveDistanceToSpeaker)),
            feedShotSpeedMap.get(effectiveDistanceToAmp),
            new Rotation2d(feedShotAngleMap.get(effectiveDistanceToAmp)));

    SimulationManager.periodic();

    Logger.recordOutput("RobotState/Primary Poses", visionPrimaryPosesSupplier.get());
    Logger.recordOutput("RobotState/Secondary Pose", visionSecondaryPosesSupplier.get());
    Logger.recordOutput("RobotState/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/Effective Aiming Pose", new Pose2d(effectiveAimingPose, new Rotation2d()));
    Logger.recordOutput("RobotState/Effective Distance To Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "RobotState/StateCache/Speaker Turret Angle", stateCache.speakerTurretAngle());
    Logger.recordOutput("RobotState/StateCache/Feed Turret Angle", stateCache.feedTurretAngle());
    Logger.recordOutput("RobotState/StateCache/Amp Turret Angle", stateCache.ampTurretAngle());
    Logger.recordOutput("RobotState/StateCache/Speaker Shot Speed", stateCache.speakerShotSpeed());
    Logger.recordOutput("RobotState/StateCache/Speaker Hood Angle", stateCache.speakerHoodAngle());
    Logger.recordOutput("RobotState/StateCache/Feed Shot Speed", stateCache.feedShotSpeed());
    Logger.recordOutput("RobotState/StateCache/Feed Hood Angle", stateCache.feedHoodAngle());
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(robotHeadingSupplier.get(), modulePositionSupplier.get(), pose);
  }

  public static Rotation2d getTargetGyroOffset(Pose2d targetPose) {
    return Rotation2d.fromRadians(
        Math.atan2(
            targetPose.getY() - getRobotPose().getY(), targetPose.getX() - getRobotPose().getX()));
  }

  public static record StateCache(
      Rotation2d speakerTurretAngle,
      Rotation2d feedTurretAngle,
      Rotation2d ampTurretAngle,
      double speakerShotSpeed,
      Rotation2d speakerHoodAngle,
      double feedShotSpeed,
      Rotation2d feedHoodAngle) {}
}

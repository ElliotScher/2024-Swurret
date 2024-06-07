package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.drive.Drive;
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
          0.0,
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          0.0,
          new Rotation2d(),
          0.0,
          new Rotation2d());

  @Getter @Setter private static double speakerFlywheelOffset = 0.0;
  @Getter @Setter private static double speakerShotOffset = 0.0;

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
    speakerShotSpeedMap.put(2.16, 800.0);
    speakerShotSpeedMap.put(4.29, 900.0);

    // Units: radians per second
    feedShotSpeedMap.put(0.0, 0.0);
    feedShotSpeedMap.put(0.0, 0.0);

    // Units: radians
    speakerShotAngleMap.put(2.16, 0.05);
    speakerShotAngleMap.put(4.29, 0.45);

    // Units: radians
    feedShotAngleMap.put(0.0, 0.0);
    feedShotAngleMap.put(0.0, 0.0);

    // Units: seconds
    timeOfFlightMap.put(2.50, (4.42 - 4.24));
    timeOfFlightMap.put(4.0, (2.60 - 2.32));
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
    Translation2d ampPose = AllianceFlipUtil.apply(FieldConstants.Amp.ampTapeTopCorner);
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

    Rotation2d speakerSetpointAngle = speakerPose.minus(effectiveAimingPose).getAngle();
    double tangentialVelocity =
        -robotFieldRelativeVelocitySupplier
            .get()
            .rotateBy(speakerSetpointAngle.unaryMinus())
            .getY();
    double radialVelocity = tangentialVelocity / effectiveDistanceToSpeaker;
    Rotation2d ampSetpointAngle = ampPose.minus(effectiveAimingPose).getAngle();
    stateCache =
        new StateCache(
            radialVelocity,
            speakerSetpointAngle,
            ampSetpointAngle,
            ampSetpointAngle,
            speakerShotSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(speakerShotAngleMap.get(effectiveDistanceToSpeaker)),
            feedShotSpeedMap.get(effectiveDistanceToAmp),
            new Rotation2d(feedShotAngleMap.get(effectiveDistanceToAmp)));

    Logger.recordOutput("RobotState/Primary Poses", visionPrimaryPosesSupplier.get());
    Logger.recordOutput("RobotState/Secondary Pose", visionSecondaryPosesSupplier.get());
    Logger.recordOutput("RobotState/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput("RobotState/StateCache/Robot Angle Setpoint", speakerSetpointAngle);
    Logger.recordOutput(
        "RobotState/StateCache/Effective Distance to Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput("RobotState/StateCache/Effective Distance to Amp", effectiveDistanceToAmp);
    Logger.recordOutput(
        "RobotState/StateCache/Effective Aiming Pose",
        new Pose2d(effectiveAimingPose, new Rotation2d()));
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Drive drive, Pose2d pose) {
    poseEstimator.resetPosition(robotHeadingSupplier.get(), modulePositionSupplier.get(), pose);
    drive.setPose(pose);
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
      double radialVelocity,
      Rotation2d speakerTurretAngle,
      Rotation2d feedTurretAngle,
      Rotation2d ampTurretAngle,
      double speakerShotSpeed,
      Rotation2d speakerHoodAngle,
      double feedShotSpeed,
      Rotation2d feedHoodAngle) {}
}

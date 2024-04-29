package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static final double BUFFER_SECONDS = 3.0;
  private static final InterpolatingDoubleTreeMap shooterSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static double lastValidTimeStamp = Double.NEGATIVE_INFINITY;
  private static Pose2d lastValidRobotPose = new Pose2d();

  private static final TimeInterpolatableBuffer<Pose2d> robotPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_SECONDS);

  private static SwerveDrivePoseEstimator poseEstimator;

  @Setter private static Supplier<Rotation2d> robotHeadingSupplier;
  @Setter private static Supplier<SwerveModulePosition[]> modulePositionSupplier;
  @Setter private static Supplier<Pose3d> visionPoseSupplier;
  @Setter private static Supplier<Pose2d> drivePoseSupplier;
  @Setter private static BooleanSupplier visionValidTargetSupplier;
  @Setter private static DoubleSupplier visionTimestampSupplier;
  @Setter private static Supplier<Rotation2d> visionXSupplier;
  @Setter private static Supplier<Rotation2d> visionYSupplier;

  static {
    // Units: radians per second
    shooterSpeedMap.put(2.16, 800.0);
    shooterSpeedMap.put(2.45, 800.0);
    shooterSpeedMap.put(2.69, 800.0);
    shooterAngleMap.put(2.84, 800.0);
    shooterSpeedMap.put(3.19, 800.0);
    shooterSpeedMap.put(3.52, 800.0);
    shooterSpeedMap.put(3.85, 900.0);
    shooterSpeedMap.put(4.29, 900.0);

    // Units: radians
    shooterAngleMap.put(2.16, 0.05);
    shooterAngleMap.put(2.45, 0.05);
    shooterAngleMap.put(2.69, 0.16);
    shooterAngleMap.put(2.84, 0.32);
    shooterAngleMap.put(3.19, 0.39);
    shooterAngleMap.put(3.52, 0.45);
    shooterAngleMap.put(3.85, 0.44);
    shooterAngleMap.put(4.29, 0.45);

    // Units: seconds
    timeOfFlightMap.put(2.50, (4.42 - 4.24));
    timeOfFlightMap.put(2.75, (2.56 - 2.33));
    timeOfFlightMap.put(3.00, (3.43 - 3.18));
    timeOfFlightMap.put(3.25, (3.20 - 2.94));
    timeOfFlightMap.put(3.50, (2.64 - 2.42));
    timeOfFlightMap.put(4.0, (2.60 - 2.32));
  }

  private RobotState() {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(Drive.getModuleTranslations()),
            robotHeadingSupplier.get(),
            modulePositionSupplier.get(),
            new Pose2d());
  }

  @Override
  public void periodic() {
    if (visionValidTargetSupplier.getAsBoolean()) {
      lastValidTimeStamp = visionTimestampSupplier.getAsDouble();
      lastValidRobotPose = visionPoseSupplier.get().toPose2d();
    }
    robotPoseBuffer.addSample(Timer.getFPGATimestamp(), drivePoseSupplier.get());

    poseEstimator.update(robotHeadingSupplier.get(), modulePositionSupplier.get());
    poseEstimator.addVisionMeasurement(visionPoseSupplier.get().toPose2d(), Timer.getFPGATimestamp());
  }

  public static Optional<Rotation2d> getTargetGyroAngle() {
    Optional<Pose2d> robotPose = robotPoseBuffer.getSample(visionTimestampSupplier.getAsDouble());
    if (robotPose.isPresent() && visionValidTargetSupplier.getAsBoolean()) {
      return Optional.of(robotPose.get().getRotation().minus(visionXSupplier.get()));
    } else {
      return Optional.empty();
    }
  }

  public static Optional<Pose2d> getRobotPose() {
    if ((Timer.getFPGATimestamp() - lastValidTimeStamp) <= BUFFER_SECONDS
        && robotPoseBuffer.getSample(lastValidTimeStamp).isPresent()) {
      Pose2d currentPoseFromDrive = drivePoseSupplier.get();
      Pose2d capturePoseFromDrive = robotPoseBuffer.getSample(lastValidTimeStamp).get();
      Pose2d capturePoseFromCam = lastValidRobotPose;

      Pose2d currentPoseFromCam =
          capturePoseFromCam.plus(currentPoseFromDrive.minus(capturePoseFromDrive));
      return Optional.of(currentPoseFromCam);
    }
    return Optional.empty();
  }

  public static AimingParameters poseCalculation(Translation2d fieldRelativeVelocity) {
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    double distanceToSpeaker =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(speakerPose);
    Translation2d effectiveAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(fieldRelativeVelocity.times(timeOfFlightMap.get(distanceToSpeaker)));
    double effectiveDistanceToSpeaker = effectiveAimingPose.getDistance(speakerPose);

    Rotation2d setpointAngle = speakerPose.minus(effectiveAimingPose).getAngle();
    double tangentialVelocity = -fieldRelativeVelocity.rotateBy(setpointAngle.unaryMinus()).getY();
    double radialVelocity = tangentialVelocity / effectiveDistanceToSpeaker;
    Logger.recordOutput("RobotState/effectiveDistanceToSpeaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "RobotState/effectiveAimingPose", new Pose2d(effectiveAimingPose, new Rotation2d()));
    Logger.recordOutput("RobotState/robotAngle", setpointAngle);
    return new AimingParameters(
        setpointAngle,
        radialVelocity,
        shooterSpeedMap.get(effectiveDistanceToSpeaker),
        new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)));
  }

  public static boolean shooterReady(Hood hood, Shooter shooter) {
    return shooter.atGoal() && hood.atGoal();
  }

  public static record AimingParameters(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}
}

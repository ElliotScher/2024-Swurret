package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static final double BUFFER_SECONDS = 3.0;
  private static final InterpolatingDoubleTreeMap shooterSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final TimeInterpolatableBuffer<Pose2d> robotPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_SECONDS);

  private static SwerveDrivePoseEstimator poseEstimator;

  private static Supplier<Rotation2d> robotHeadingSupplier;
  private static Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private static Supplier<Pose3d[]> visionMegaTag1Supplier;
  private static Supplier<Pose3d[]> visionMegaTag2Supplier;
  private static Supplier<Pose2d> drivePoseSupplier;
  private static Supplier<double[]> visionMegaTag1TimestampSupplier;
  private static Supplier<double[]> visionMegaTag2TimestampSupplier;

  @Getter private static double flywheelOffset = 0.0;
  @Getter private static double hoodOffset = 0.0;

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

  public RobotState(
      Supplier<Rotation2d> robotHeadingSupplier,
      Supplier<SwerveModulePosition[]> modulePositionSupplier,
      Supplier<Pose3d[]> visionMegaTag1Supplier,
      Supplier<Pose3d[]> visionMegaTag2Supplier,
      Supplier<Pose2d> drivePoseSupplier,
      Supplier<double[]> visionMegaTag1TimestampSupplier,
      Supplier<double[]> visionMegaTag2TimestampSupplier) {
    RobotState.robotHeadingSupplier = robotHeadingSupplier;
    RobotState.modulePositionSupplier = modulePositionSupplier;
    RobotState.visionMegaTag1Supplier = visionMegaTag1Supplier;
    RobotState.visionMegaTag2Supplier = visionMegaTag2Supplier;
    RobotState.drivePoseSupplier = drivePoseSupplier;
    RobotState.visionMegaTag1TimestampSupplier = visionMegaTag1TimestampSupplier;
    RobotState.visionMegaTag2TimestampSupplier = visionMegaTag2TimestampSupplier;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            robotHeadingSupplier.get(),
            modulePositionSupplier.get(),
            new Pose2d(),
            DriveConstants.ODOMETRY_STANDARD_DEVIATIONS,
            VisionConstants.MEGA_TAG_DEFAULT_STANDARD_DEVIATIONS);
  }

  @Override
  public void periodic() {
    robotPoseBuffer.addSample(Timer.getFPGATimestamp(), drivePoseSupplier.get());

    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), robotHeadingSupplier.get(), modulePositionSupplier.get());
    for (int i = 0; i < visionMegaTag1Supplier.get().length; i++) {
      poseEstimator.addVisionMeasurement(
          visionMegaTag1Supplier.get()[i].toPose2d(),
          visionMegaTag1TimestampSupplier.get()[i],
          VisionConstants.MEGA_TAG_1_STANDARD_DEVIATIONS);
    }
    for (int i = 0; i < visionMegaTag2Supplier.get().length; i++) {
      poseEstimator.addVisionMeasurement(
          visionMegaTag2Supplier.get()[i].toPose2d(),
          visionMegaTag2TimestampSupplier.get()[i],
          VisionConstants.MEGA_TAG_2_STANDARD_DEVIATIONS);
    }

    Logger.recordOutput("RobotState/MegaTag 1 Pose", visionMegaTag1Supplier.get());
    Logger.recordOutput("RobotState/MegaTag 2 Pose", visionMegaTag2Supplier.get());
    Logger.recordOutput("RobotState/Estimated Pose", poseEstimator.getEstimatedPosition());
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(robotHeadingSupplier.get(), modulePositionSupplier.get(), pose);
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
    Logger.recordOutput(
        "RobotState/AimingParameters/Effective Distance to Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "RobotState/AimingParameters/Effective Aiming Pose",
        new Pose2d(effectiveAimingPose, new Rotation2d()));
    Logger.recordOutput("RobotState/AimingParameters/Robot Angle", setpointAngle);
    return new AimingParameters(
        setpointAngle,
        radialVelocity,
        shooterSpeedMap.get(effectiveDistanceToSpeaker),
        new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)));
  }

  public static Rotation2d getTargetGyroAngle(Pose2d targetPose) {
    return new Transform2d(getRobotPose(), AllianceFlipUtil.apply(targetPose)).getRotation();
  }

  public static boolean shooterReady(Hood hood, Shooter shooter) {
    return shooter.atGoal() && hood.atGoal();
  }

  public static Command increaseFlywheelVelocity() {
    return Commands.runOnce(() -> flywheelOffset += 10);
  }

  public static Command decreaseFlywheelVelocity() {
    return Commands.runOnce(() -> flywheelOffset -= 10);
  }

  public static Command increaseHoodAngle() {
    return Commands.runOnce(() -> hoodOffset += Units.degreesToRadians(0.25));
  }

  public static Command decreaseHoodAngle() {
    return Commands.runOnce(() -> hoodOffset -= Units.degreesToRadians(0.25));
  }

  public static record AimingParameters(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}
}

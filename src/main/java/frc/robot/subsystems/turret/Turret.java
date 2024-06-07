package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final ProfiledPIDController profiledFeedback =
      new ProfiledPIDController(
          TurretConstants.KP.get(),
          0.0,
          TurretConstants.KD.get(),
          new Constraints(
              TurretConstants.MAX_VELOCITY.get(), TurretConstants.MAX_ACCELERATION.get()));
  private final TurretIO io;

  public Turret(TurretIO io) {
    this.io = io;
    setDefaultCommand(
        run(
            () -> {
              setShootPosition();
            }));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (TurretConstants.KP.hasChanged(hashCode())) {
      profiledFeedback.setP(TurretConstants.KP.get());
    }
    if (TurretConstants.KD.hasChanged(hashCode())) {
      profiledFeedback.setD(TurretConstants.KD.get());
    }
    if (TurretConstants.MAX_VELOCITY.hasChanged(hashCode())
        || TurretConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
      profiledFeedback.setConstraints(
          new Constraints(
              TurretConstants.MAX_VELOCITY.get(), TurretConstants.MAX_ACCELERATION.get()));
    }

    if (DriverStation.isEnabled()) {
      io.setVoltage(profiledFeedback.calculate(inputs.position.getRadians()));
    }

    if (DriverStation.isDisabled()) {
      profiledFeedback.reset(inputs.position.getRadians(), 0);
    }

    Logger.recordOutput("Turret/Goal", profiledFeedback.getGoal().position);
    Logger.recordOutput("Turret/Setpoint", profiledFeedback.getSetpoint().position);
  }

  // Normalize the angle to be within [-(2.0 * Math.PI), (2.0 * Math.PI))
  private double normalizeAngle(double angle) {
    angle = angle % (4.0 * Math.PI);
    if (angle < -(2.0 * Math.PI)) {
      angle += (4.0 * Math.PI);
    } else if (angle >= (2.0 * Math.PI)) {
      angle -= (4.0 * Math.PI);
    }
    return angle;
  }

  // Calculate the shortest rotation direction and angle
  private double calculateShortestRotation(double currentPosition, double targetPosition) {
    currentPosition = normalizeAngle(currentPosition);
    targetPosition = normalizeAngle(targetPosition);

    // Direct difference
    double directDiff = targetPosition - currentPosition;

    // Calculate both possible rotations
    double clockwiseRotation = directDiff >= 0 ? directDiff : directDiff + (4.0 * Math.PI);
    double counterclockwiseRotation = directDiff <= 0 ? directDiff : directDiff - (4.0 * Math.PI);

    // Choose the shortest path within [-(2.0 * Math.PI), (2.0 * Math.PI)]
    return Math.abs(clockwiseRotation) <= Math.abs(counterclockwiseRotation)
        ? clockwiseRotation
        : counterclockwiseRotation;
  }

  // Calculate the final position ensuring movement within [-(2.0 * Math.PI), (2.0 * Math.PI)]
  private double calculateFinalPosition(double currentPosition, double targetPosition) {
    double rotation = calculateShortestRotation(currentPosition, targetPosition);
    double finalPosition = currentPosition + rotation;
    // Normalize the final position within [-(2.0 * Math.PI), (2.0 * Math.PI))
    finalPosition = normalizeAngle(finalPosition);

    return finalPosition;
  }

  public Rotation2d getPosition() {
    return inputs.position;
  }

  public Command setShootPosition() {
    return Commands.runOnce(
        () ->
            profiledFeedback.setGoal(
                calculateFinalPosition(
                    inputs.position.getRadians(),
                    RobotState.getStateCache().speakerTurretAngle().getRadians())));
  }

  public Command setFeedPosition() {
    return Commands.runOnce(
        () ->
            profiledFeedback.setGoal(
                calculateFinalPosition(
                    inputs.position.getRadians(),
                    RobotState.getStateCache().feedTurretAngle().getRadians())));
  }

  public Command setAmpPosition() {
    return Commands.runOnce(
        () ->
            profiledFeedback.setGoal(
                calculateFinalPosition(
                    inputs.position.getRadians(),
                    RobotState.getStateCache().ampTurretAngle().getRadians())));
  }
}

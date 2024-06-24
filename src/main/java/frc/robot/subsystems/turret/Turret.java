package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.HoodConstants;
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
        Commands.run(
            () -> {
              setFeedPosition();
            },
            this));
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
    Logger.recordOutput("Turret/Sine", inputs.position.getSin());
    Logger.recordOutput("Turret/Cosine", inputs.position.getCos());
  }

  private void setPosition(double targetAngleRad) {
    double currentAngleRad = inputs.position.getRadians();
    double normalizedTargetAngleRad = normalizeAngle(targetAngleRad);
    double shortestPathAngleRad = calculateShortestPath(currentAngleRad, normalizedTargetAngleRad);
    profiledFeedback.setGoal(shortestPathAngleRad);
  }

  private double normalizeAngle(double angleRad) {
    return ((angleRad % (2.0 * Math.PI)) + (2.0 * Math.PI)) % (2.0 * Math.PI);
  }

  private double calculateShortestPath(double currentAngleRad, double targetAngleRad) {
    double deltaAngleRad = targetAngleRad - currentAngleRad;
    deltaAngleRad = Math.atan2(Math.sin(deltaAngleRad), Math.cos(deltaAngleRad));
    return currentAngleRad + deltaAngleRad;
  }

  public boolean atGoal() {
    return Math.abs(profiledFeedback.getGoal().position - profiledFeedback.getSetpoint().position)
        <= HoodConstants.GOAL_TOLERANCE.get();
  }

  public Rotation2d getPosition() {
    return inputs.position;
  }

  public Command setShootPosition() {
    return Commands.run(
        () -> setPosition(RobotState.getShotCache().speakerTurretAngle().getRadians()));
  }

  public Command setAmpPosition() {
    return Commands.run(
        () -> setPosition(RobotState.getShotCache().ampTurretAngle().getRadians()));
  }

  public Command setFeedPosition() {
    return Commands.run(
        () -> setPosition(RobotState.getShotCache().feedTurretAngle().getRadians()));
  }
}

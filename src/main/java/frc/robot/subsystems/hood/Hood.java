package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final ProfiledPIDController profiledFeedback =
      new ProfiledPIDController(
          HoodConstants.KP.get(),
          0.0,
          HoodConstants.KD.get(),
          new Constraints(HoodConstants.MAX_VELOCITY.get(), HoodConstants.MAX_ACCELERATION.get()));
  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
    setDefaultCommand(
        Commands.run(
            () -> {
              setPosition(HoodConstants.STOWED_POSITION.get());
            },
            this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (HoodConstants.KP.hasChanged(hashCode())) {
      profiledFeedback.setP(HoodConstants.KP.get());
    }
    if (HoodConstants.KD.hasChanged(hashCode())) {
      profiledFeedback.setD(HoodConstants.KD.get());
    }
    if (HoodConstants.MAX_VELOCITY.hasChanged(hashCode())
        || HoodConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
      profiledFeedback.setConstraints(
          new Constraints(HoodConstants.MAX_VELOCITY.get(), HoodConstants.MAX_ACCELERATION.get()));
    }

    if (DriverStation.isEnabled()) {
      io.setVoltage(profiledFeedback.calculate(inputs.position.getRadians()));
    }

    if (DriverStation.isDisabled()) {
      profiledFeedback.reset(inputs.position.getRadians(), 0);
    }

    Logger.recordOutput("Hood/Goal", profiledFeedback.getGoal().position);
    Logger.recordOutput("Hood/Setpoint", profiledFeedback.getSetpoint().position);
  }

  private void setPosition(double positionRad) {
    profiledFeedback.setGoal(positionRad + RobotState.getSpeakerAngleCompensation());
  }

  public Rotation2d getPosition() {
    return inputs.position;
  }

  public boolean atGoal() {
    return Math.abs(profiledFeedback.getGoal().position - profiledFeedback.getSetpoint().position)
        <= HoodConstants.GOAL_TOLERANCE.get();
  }

  public Command setSpeakerPosition() {
    return Commands.run(
        () -> setPosition(RobotState.getControlData().speakerHoodAngle().getRadians()));
  }

  public Command setFeedPosition() {
    return Commands.run(
        () -> setPosition(RobotState.getControlData().feedHoodAngle().getRadians()));
  }

  public Command setAmpPosition() {
    return Commands.run(() -> setPosition(HoodConstants.AMP_POSITION.get()));
  }
}

package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final ProfiledPIDController leftProfiledFeedback;
  private final ProfiledPIDController rightProfiledFeedback;

  public Climber(ClimberIO io) {
    this.io = io;
    leftProfiledFeedback =
        new ProfiledPIDController(
            ClimberConstants.KP.get(),
            0.0,
            ClimberConstants.KD.get(),
            new Constraints(
                ClimberConstants.MAX_VELOCITY.get(), ClimberConstants.MAX_ACCELERATION.get()));
    rightProfiledFeedback =
        new ProfiledPIDController(
            ClimberConstants.KP.get(),
            0.0,
            ClimberConstants.KD.get(),
            new Constraints(
                ClimberConstants.MAX_VELOCITY.get(), ClimberConstants.MAX_ACCELERATION.get()));

    leftProfiledFeedback.setTolerance(ClimberConstants.CLIMB_POSITION_TOLERANCE);
    rightProfiledFeedback.setTolerance(ClimberConstants.CLIMB_POSITION_TOLERANCE);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (ClimberConstants.KP.hasChanged(hashCode())) {
      leftProfiledFeedback.setP(ClimberConstants.KP.get());
      rightProfiledFeedback.setP(ClimberConstants.KP.get());
    }
    if (ClimberConstants.KD.hasChanged(hashCode())) {
      leftProfiledFeedback.setD(ClimberConstants.KD.get());
      rightProfiledFeedback.setD(ClimberConstants.KD.get());
    }
    if (ClimberConstants.MAX_VELOCITY.hasChanged(hashCode())
        || ClimberConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
      leftProfiledFeedback.setConstraints(
          new Constraints(
              ClimberConstants.MAX_VELOCITY.get(), ClimberConstants.MAX_ACCELERATION.get()));
      rightProfiledFeedback.setConstraints(
          new Constraints(
              ClimberConstants.MAX_VELOCITY.get(), ClimberConstants.MAX_ACCELERATION.get()));
    }

    if (DriverStation.isDisabled()) {
      io.setLock(true);
      leftProfiledFeedback.reset(inputs.leftPositionMeters, 0.0);
      rightProfiledFeedback.reset(inputs.rightPositionMeters, 0.0);
    }

    Logger.recordOutput("Climber/Left/Goal", leftProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Left/Setpoint", leftProfiledFeedback.getSetpoint().position);
    Logger.recordOutput("Climber/Right/Goal", rightProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Right/Setpoint", rightProfiledFeedback.getSetpoint().position);
  }

  private Command setLeftPosition(double leftPositionMeters) {
    return Commands.run(
            () ->
                io.setLeftVoltage(
                    leftProfiledFeedback.calculate(inputs.leftPositionMeters, leftPositionMeters)))
        .until(() -> leftProfiledFeedback.atGoal());
  }

  private Command setRightPosition(double rightPositionMeters) {
    return Commands.run(
            () ->
                io.setRightVoltage(
                    rightProfiledFeedback.calculate(
                        inputs.rightPositionMeters, rightPositionMeters)))
        .until(() -> rightProfiledFeedback.atGoal());
  }

  public double getLeftPositionMeters() {
    return inputs.leftPositionMeters;
  }

  public double getRightPositionMeters() {
    return inputs.rightPositionMeters;
  }

  public Command preClimb() {
    return Commands.runOnce(() -> io.setLock(false))
        .andThen(Commands.waitSeconds(ClimberConstants.LOCKING_DELAY))
        .andThen(
            setLeftPosition(ClimberConstants.CLIMB_POSITION.get())
                .alongWith(setRightPosition(ClimberConstants.CLIMB_POSITION.get())))
        .finallyDo(() -> io.setLock(true));
  }

  public Command climbAutomatic() {
    return Commands.sequence(
        Commands.runOnce(() -> io.setLock(false)),
        Commands.waitSeconds(ClimberConstants.LOCKING_DELAY),
        Commands.parallel(
            Commands.runEnd(
                    () -> io.setLeftVoltage(ClimberConstants.CLIMB_STAGE_1_VOLTAGE),
                    () -> io.setLeftVoltage(0.0))
                .until(
                    () ->
                        inputs.leftCurrentAmps[inputs.leftCurrentAmps.length - 1]
                            >= ClimberConstants.CLIMB_TRANSITION_CURRENT_THRESHOLD),
            Commands.runEnd(
                    () -> io.setRightVoltage(ClimberConstants.CLIMB_STAGE_1_VOLTAGE),
                    () -> io.setRightVoltage(0.0))
                .until(
                    () ->
                        inputs.rightCurrentAmps[inputs.rightCurrentAmps.length - 1]
                            >= ClimberConstants.CLIMB_TRANSITION_CURRENT_THRESHOLD)),
        Commands.race(
            Commands.runEnd(
                    () -> io.setLeftVoltage(ClimberConstants.CLIMB_STAGE_2_VOLTAGE),
                    () -> io.setLeftVoltage(0.0))
                .until(
                    () ->
                        inputs.leftPositionMeters < ClimberConstants.CLIMB_STOP_POSITION_THRESHOLD),
            Commands.runEnd(
                    () -> io.setRightVoltage(ClimberConstants.CLIMB_STAGE_2_VOLTAGE),
                    () -> io.setRightVoltage(0.0))
                .until(
                    () ->
                        inputs.leftPositionMeters
                            < ClimberConstants.CLIMB_STOP_POSITION_THRESHOLD)),
        Commands.parallel(
                Commands.run(
                    () -> {
                      io.setLeftVoltage(ClimberConstants.CLIMB_HOLD_VOLTAGE);
                      io.setRightVoltage(ClimberConstants.CLIMB_HOLD_VOLTAGE);
                    }),
                Commands.runOnce(() -> io.setLock(true)))
            .withTimeout(ClimberConstants.LOCKING_DELAY),
        stop(),
        Commands.runOnce(
            () -> {
              leftProfiledFeedback.reset(
                  inputs.leftPositionMeters, inputs.leftVelocityMetersPerSec);
              rightProfiledFeedback.reset(
                  inputs.leftPositionMeters, inputs.leftVelocityMetersPerSec);
            }));
  }

  public Command zero() {
    return Commands.sequence(
        Commands.runOnce(() -> io.setLock(false)),
        Commands.waitSeconds(ClimberConstants.LOCKING_DELAY),
        Commands.parallel(
            Commands.runEnd(
                    () -> io.setLeftVoltage(ClimberConstants.CLIMB_ZERO_VOLTAGE),
                    () -> io.setLeftVoltage(0.0))
                .until(
                    () ->
                        inputs.leftCurrentAmps[inputs.leftCurrentAmps.length - 1]
                            >= ClimberConstants.CLIMB_ZERO_CURRENT_THRESHOLD),
            Commands.runEnd(
                    () -> io.setRightVoltage(ClimberConstants.CLIMB_ZERO_VOLTAGE),
                    () -> io.setRightVoltage(0.0))
                .until(
                    () ->
                        inputs.rightCurrentAmps[inputs.rightCurrentAmps.length - 1]
                            >= ClimberConstants.CLIMB_ZERO_CURRENT_THRESHOLD)),
        stop(),
        Commands.runOnce(
            () -> {
              io.setLock(true);
              io.resetPosition();
              leftProfiledFeedback.reset(0.0, inputs.leftVelocityMetersPerSec);
              rightProfiledFeedback.reset(0.0, inputs.leftVelocityMetersPerSec);
            }));
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setLeftVoltage(0.0);
          io.setRightVoltage(0.0);
          io.setLock(true);
        });
  }
}

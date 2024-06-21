package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LinearProfile;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(ShooterConstants.RAMP_RATE_VOLTAGE)
                  .per(Seconds.of(ShooterConstants.RAMP_RATE_SECONDS)),
              Volts.of(ShooterConstants.STEP_VOLTAGE),
              Seconds.of(ShooterConstants.SYSID_TIMEOUT),
              (state) -> Logger.recordOutput("Shooter/sysID State", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));
  private static SimpleMotorFeedforward leftFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.KS_LEFT.get(),
          ShooterConstants.KV_LEFT.get(),
          ShooterConstants.KA_LEFT.get());
  private static SimpleMotorFeedforward rightFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.KS_RIGHT.get(),
          ShooterConstants.KV_RIGHT.get(),
          ShooterConstants.KA_RIGHT.get());
  private static PIDController leftFeedback =
      new PIDController(
          ShooterConstants.KP.get(), 0.0, ShooterConstants.KD.get(), Constants.LOOP_PERIOD_SECS);
  private static PIDController rightFeedback =
      new PIDController(
          ShooterConstants.KP.get(), 0.0, ShooterConstants.KD.get(), Constants.LOOP_PERIOD_SECS);
  private static LinearProfile leftProfile =
      new LinearProfile(ShooterConstants.MAX_ACCELERATION.get(), Constants.LOOP_PERIOD_SECS);
  private static LinearProfile rightProfile =
      new LinearProfile(ShooterConstants.MAX_ACCELERATION.get(), Constants.LOOP_PERIOD_SECS);

  @AutoLogOutput(key = "Shooter/Spin Direction")
  private SpinDirection spinDirection = SpinDirection.CLOCKWISE;

  private boolean isOpenLoop = true;
  private double openLoopVoltage = 0.0;

  @Getter private boolean isShooting = false;

  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
    setDefaultCommand(runVelocity());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (ShooterConstants.KP.hasChanged(hashCode())) {
      leftFeedback.setP(ShooterConstants.KP.get());
      rightFeedback.setP(ShooterConstants.KP.get());
    }
    if (ShooterConstants.KD.hasChanged(hashCode())) {
      leftFeedback.setD(ShooterConstants.KD.get());
      rightFeedback.setD(ShooterConstants.KD.get());
    }
    if (ShooterConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
      leftProfile.setMaxAcceleration(ShooterConstants.MAX_ACCELERATION.get());
      rightProfile.setMaxAcceleration(ShooterConstants.MAX_ACCELERATION.get());
    }
    if (ShooterConstants.KS_LEFT.hasChanged(hashCode())
        || ShooterConstants.KV_LEFT.hasChanged(hashCode())
        || ShooterConstants.KA_LEFT.hasChanged(hashCode())) {
      leftFeedforward =
          new SimpleMotorFeedforward(
              ShooterConstants.KS_LEFT.get(),
              ShooterConstants.KV_LEFT.get(),
              ShooterConstants.KA_LEFT.get());
    }
    if (ShooterConstants.KS_RIGHT.hasChanged(hashCode())
        || ShooterConstants.KV_RIGHT.hasChanged(hashCode())
        || ShooterConstants.KA_RIGHT.hasChanged(hashCode())) {
      rightFeedforward =
          new SimpleMotorFeedforward(
              ShooterConstants.KS_RIGHT.get(),
              ShooterConstants.KV_RIGHT.get(),
              ShooterConstants.KA_RIGHT.get());
    }

    if (!isOpenLoop) {
      double leftSetpoint = leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      leftFeedback.setSetpoint(leftSetpoint);
      rightFeedback.setSetpoint(rightSetpoint);
      io.setLeftVoltage(
          leftFeedforward.calculate(leftSetpoint)
              + leftFeedback.calculate(inputs.leftVelocityRadPerSec));
      io.setRightVoltage(
          rightFeedforward.calculate(rightSetpoint)
              + rightFeedback.calculate(inputs.rightVelocityRadPerSec));
    } else {
      io.setLeftVoltage(openLoopVoltage);
      io.setRightVoltage(openLoopVoltage);
    }

    Logger.recordOutput("Shooter/Left Goal", leftProfile.getGoal());
    Logger.recordOutput("Shooter/Right Goal", rightProfile.getGoal());

    Logger.recordOutput("Shooter/Left Setpoint", leftProfile.calculateSetpoint());
    Logger.recordOutput("Shooter/Right Setpoint", rightProfile.calculateSetpoint());

    Logger.recordOutput("Shooter/At Goal", atGoal());
  }

  private void setSpinVelocity(double velocityRadPerSec) {
    isOpenLoop = false;
    if (spinDirection.equals(SpinDirection.COUNTERCLOCKWISE)) {
      leftProfile.setGoal(
          (velocityRadPerSec + RobotState.getSpeakerFlywheelCompensation())
              * (ShooterConstants.RATIO.get()));
      rightProfile.setGoal(velocityRadPerSec + RobotState.getSpeakerFlywheelCompensation());
    } else if (spinDirection.equals(SpinDirection.CLOCKWISE)) {
      leftProfile.setGoal(velocityRadPerSec + RobotState.getSpeakerFlywheelCompensation());
      rightProfile.setGoal(
          (velocityRadPerSec + RobotState.getSpeakerFlywheelCompensation())
              * (ShooterConstants.RATIO.get()));
    } else {
      leftProfile.setGoal(velocityRadPerSec + RobotState.getSpeakerFlywheelCompensation());
      rightProfile.setGoal(velocityRadPerSec + RobotState.getSpeakerFlywheelCompensation());
    }
  }

  private void stop() {
    isOpenLoop = true;
    openLoopVoltage = 0.0;
  }

  private void setVoltage(double volts) {
    isOpenLoop = true;
    openLoopVoltage = volts;
  }

  public double getSpeed() {
    return Math.max(inputs.leftVelocityRadPerSec, inputs.rightVelocityRadPerSec);
  }

  public double getLeftSpeed() {
    return inputs.leftVelocityRadPerSec;
  }

  public double getRightSpeed() {
    return inputs.rightVelocityRadPerSec;
  }

  public boolean atGoal() {
    return (Math.abs(leftProfile.getGoal() - leftFeedback.getSetpoint())
            <= ShooterConstants.GOAL_TOLERANCE.get())
        && (Math.abs(rightProfile.getGoal() - rightFeedback.getSetpoint())
            <= ShooterConstants.GOAL_TOLERANCE.get());
  }

  public Command runVelocity() {
    return Commands.runEnd(
        () -> {
          setSpinVelocity(ShooterConstants.IDLE_SPEED.get());
        },
        () -> {
          stop();
        },
        this);
  }

  public Command setDefaultSpeed() {
    return Commands.run(() -> setSpinVelocity(ShooterConstants.IDLE_SPEED.get()));
  }

  public Command setSpeakerSpeed() {
    return Commands.run(
            () -> {
              isShooting = true;
              setSpinVelocity(RobotState.getStateCache().speakerShotSpeed());
            })
        .finallyDo(
            () -> {
              isShooting = false;
            });
  }

  public Command setFeedSpeed() {
    return Commands.run(
        () -> {
          setSpinVelocity(RobotState.getStateCache().feedShotSpeed());
        });
  }

  public Command setAmpSpeed() {
    return Commands.run(
        () -> {
          setSpinVelocity(ShooterConstants.AMP_SPEED.get());
        });
  }

  public Command runSysId() {
    return Commands.sequence(
        sysIdRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(ShooterConstants.SYSID_DELAY),
        sysIdRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(ShooterConstants.SYSID_DELAY),
        sysIdRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(ShooterConstants.SYSID_DELAY),
        sysIdRoutine.dynamic(Direction.kReverse));
  }
}

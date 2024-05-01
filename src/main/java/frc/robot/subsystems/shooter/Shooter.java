package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.AimingParameters;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LinearProfile;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SimpleMotorFeedforward leftFeedforward;
  private static SimpleMotorFeedforward rightFeedforward;

  private static LinearProfile leftProfile;
  private static LinearProfile rightProfile;

  private static PIDController leftFeedback;
  private static PIDController rightFeedback;

  private boolean isOpenLoop = true;
  private double openLoopVoltage = 0.0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(ShooterConstants.RAMP_RATE_VOLTAGE)
                  .per(Seconds.of(ShooterConstants.RAMP_RATE_SECONDS)),
              Volts.of(ShooterConstants.STEP_VOLTAGE),
              Seconds.of(ShooterConstants.SYSID_TIMEOUT),
              (state) -> Logger.recordOutput("Shooter/sysID State", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));

  @AutoLogOutput(key = "Shooter/Spin Direction")
  private SpinDirection spinDirection = SpinDirection.CLOCKWISE;

  public Shooter(ShooterIO io) {
    this.io = io;
    leftFeedback =
        new PIDController(
            ShooterConstants.KP.get(), 0.0, ShooterConstants.KD.get(), Constants.LOOP_PERIOD_SECS);
    rightFeedback =
        new PIDController(
            ShooterConstants.KP.get(), 0.0, ShooterConstants.KD.get(), Constants.LOOP_PERIOD_SECS);

    leftFeedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.KS_LEFT.get(),
            ShooterConstants.KV_LEFT.get(),
            ShooterConstants.KA_LEFT.get());
    rightFeedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.KS_RIGHT.get(),
            ShooterConstants.KV_RIGHT.get(),
            ShooterConstants.KA_RIGHT.get());

    leftProfile =
        new LinearProfile(ShooterConstants.MAX_ACCELERATION.get(), Constants.LOOP_PERIOD_SECS);
    rightProfile =
        new LinearProfile(ShooterConstants.MAX_ACCELERATION.get(), Constants.LOOP_PERIOD_SECS);

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
          (velocityRadPerSec + RobotState.getFlywheelOffset()) * (ShooterConstants.RATIO.get()));
      rightProfile.setGoal(velocityRadPerSec + RobotState.getFlywheelOffset());
    } else if (spinDirection.equals(SpinDirection.CLOCKWISE)) {
      leftProfile.setGoal(velocityRadPerSec + RobotState.getFlywheelOffset());
      rightProfile.setGoal(
          (velocityRadPerSec + RobotState.getFlywheelOffset()) * (ShooterConstants.RATIO.get()));
    } else {
      // YEET MODE :)
      leftProfile.setGoal(velocityRadPerSec + RobotState.getFlywheelOffset());
      rightProfile.setGoal(velocityRadPerSec + RobotState.getFlywheelOffset());
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

  public boolean atGoal() {
    return (Math.abs(leftProfile.getGoal() - leftFeedback.getSetpoint())
            <= ShooterConstants.GOAL_TOLERANCE.get())
        && (Math.abs(rightProfile.getGoal() - rightFeedback.getSetpoint())
            <= ShooterConstants.GOAL_TOLERANCE.get());
  }

  public Command runVelocity() {
    return runEnd(
        () -> {
          setSpinVelocity(ShooterConstants.DEFAULT_SPEED.get());
        },
        () -> {
          stop();
        });
  }

  public Command runSourceFeed() {
    return runEnd(
        () -> {
          setSpinVelocity(ShooterConstants.SOURCE_FEED_SPEED.get());
        },
        () -> {
          stop();
        });
  }

  public Command runAmpFeed() {
    return runEnd(
        () -> {
          setSpinVelocity(ShooterConstants.AMP_FEED_SPEED.get());
        },
        () -> {
          stop();
        });
  }

  public Command runPoseDistance(
      Supplier<Translation2d> robotPoseSupplier, Supplier<Translation2d> velocitySupplier) {
    return runEnd(
        () -> {
          AimingParameters aimingParameters = RobotState.poseCalculation(velocitySupplier.get());
          if (spinDirection.equals(SpinDirection.YEET)) {
            spinDirection = SpinDirection.CLOCKWISE;
          }
          Rotation2d effectiveRobotAngle = aimingParameters.robotAngle();

          Translation2d speakerPose =
              AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
          double distanceToSpeaker = robotPoseSupplier.get().getDistance(speakerPose);

          if (distanceToSpeaker > ShooterConstants.ENABLED_SPIN_DISTANCE) {
            if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
              effectiveRobotAngle = effectiveRobotAngle.minus(Rotation2d.fromDegrees(180));
            }
            if (effectiveRobotAngle.getDegrees()
                > ShooterConstants.SPEAKER_TO_ROBOT_SPIN_THRESHOLD) {
              spinDirection = SpinDirection.COUNTERCLOCKWISE;
            } else if (effectiveRobotAngle.getDegrees()
                < -ShooterConstants.SPEAKER_TO_ROBOT_SPIN_THRESHOLD) {
              spinDirection = SpinDirection.CLOCKWISE;
            }
          }

          setSpinVelocity(RobotState.poseCalculation(velocitySupplier.get()).shooterSpeed());
        },
        () -> {
          stop();
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

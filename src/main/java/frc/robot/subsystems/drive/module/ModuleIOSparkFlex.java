// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.drive.SparkMaxOdometryThread;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {
  private final CANSparkFlex driveSparkFlex;
  private final CANSparkFlex turnSparkFlex;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;

  public ModuleIOSparkFlex(int index) {
    switch (index) {
      case 0:
        driveSparkFlex =
            new CANSparkFlex(
                ModuleConstants.frontLeftConfigREVThroughBore.drive(), MotorType.kBrushless);
        turnSparkFlex =
            new CANSparkFlex(
                ModuleConstants.frontLeftConfigREVThroughBore.turn(), MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        break;
      case 1:
        driveSparkFlex =
            new CANSparkFlex(
                ModuleConstants.frontRightConfigREVThroughBore.drive(), MotorType.kBrushless);
        turnSparkFlex =
            new CANSparkFlex(
                ModuleConstants.frontRightConfigREVThroughBore.turn(), MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        break;
      case 2:
        driveSparkFlex =
            new CANSparkFlex(
                ModuleConstants.rearLeftConfigREVThroughBore.drive(), MotorType.kBrushless);
        turnSparkFlex =
            new CANSparkFlex(
                ModuleConstants.rearLeftConfigREVThroughBore.turn(), MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        break;
      case 3:
        driveSparkFlex =
            new CANSparkFlex(
                ModuleConstants.rearRightConfigREVThroughBore.drive(), MotorType.kBrushless);
        turnSparkFlex =
            new CANSparkFlex(
                ModuleConstants.rearRightConfigREVThroughBore.turn(), MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkFlex.restoreFactoryDefaults();
    turnSparkFlex.restoreFactoryDefaults();

    driveSparkFlex.setCANTimeout(ModuleConstants.CAN_TIMEOUT);
    turnSparkFlex.setCANTimeout(ModuleConstants.CAN_TIMEOUT);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkFlex.getEncoder();

    turnSparkFlex.setInverted(isTurnMotorInverted);
    driveSparkFlex.setSmartCurrentLimit(ModuleConstants.DRIVE_CURRENT_LIMIT);
    turnSparkFlex.setSmartCurrentLimit(ModuleConstants.TURN_CURRENT_LIMIT);
    driveSparkFlex.enableVoltageCompensation(ModuleConstants.NOMINAL_VOLTAGE);
    turnSparkFlex.enableVoltageCompensation(ModuleConstants.NOMINAL_VOLTAGE);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(ModuleConstants.VELOCITY_MEASUREMENT_PERIOD);
    driveEncoder.setAverageDepth(ModuleConstants.VELOCITY_AVERAGE_DEPTH);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(ModuleConstants.VELOCITY_MEASUREMENT_PERIOD);
    turnRelativeEncoder.setAverageDepth(ModuleConstants.VELOCITY_AVERAGE_DEPTH);

    driveSparkFlex.setCANTimeout(0);
    turnSparkFlex.setCANTimeout(0);

    driveSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / ModuleConstants.ODOMETRY_FREQUENCY));
    turnSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / ModuleConstants.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = driveEncoder.getPosition();
                  if (driveSparkFlex.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkFlex.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    driveSparkFlex.burnFlash();
    turnSparkFlex.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePosition =
        Rotation2d.fromRotations(driveEncoder.getPosition() / ModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / ModuleConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsoluteEncoder.getPosition());
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / ModuleConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / ModuleConstants.TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkFlex.getAppliedOutput() * turnSparkFlex.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkFlex.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .map(
                (Double value) ->
                    Units.rotationsToRadians(value) / ModuleConstants.DRIVE_GEAR_RATIO)
            .toArray(Rotation2d[]::new);
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> Rotation2d.fromRotations(value / ModuleConstants.TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkFlex.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}

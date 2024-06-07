package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final StatusSignal<Double> leftPosition;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> leftCurrent;
  private final StatusSignal<Double> leftTemperature;

  private final StatusSignal<Double> rightPosition;
  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> rightCurrent;
  private final StatusSignal<Double> rightTemperature;

  private final Alert leftTalonDisconnectedAlert;
  private final Alert rightTalonDisconnectedAlert;

  public HoodIOTalonFX() {
    leftTalon = new TalonFX(HoodConstants.LEFT_DEVICE_ID);
    rightTalon = new TalonFX(HoodConstants.RIGHT_DEVICE_ID);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftTalon.getConfigurator().apply(motorConfig);
    rightTalon.getConfigurator().apply(motorConfig);

    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));

    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftCurrent = leftTalon.getSupplyCurrent();
    leftTemperature = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightCurrent = rightTalon.getSupplyCurrent();
    rightTemperature = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        leftTemperature,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent,
        rightTemperature);
    leftTalon.optimizeBusUtilization();

    leftTalonDisconnectedAlert =
        new Alert("Left Hood Talon is disconnected, check CAN bus", AlertType.ERROR);
    rightTalonDisconnectedAlert =
        new Alert("Right Hood Talon is disconnected, check CAN bus", AlertType.ERROR);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean leftTalonConnected =
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftCurrent, leftTemperature)
            .isOK();
    boolean rightTalonConnected =
        BaseStatusSignal.refreshAll(
                rightPosition, rightVelocity, rightAppliedVolts, rightCurrent, rightTemperature)
            .isOK();

    leftTalonDisconnectedAlert.set(!leftTalonConnected);
    rightTalonDisconnectedAlert.set(!rightTalonConnected);

    inputs.position =
        Rotation2d.fromRotations(leftPosition.getValueAsDouble() / HoodConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) / HoodConstants.GEAR_RATIO;
    inputs.appliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.currentAmps = leftCurrent.getValueAsDouble();
    inputs.tempCelcius = leftTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leftTalon.setControl(new VoltageOut(volts));
  }
}

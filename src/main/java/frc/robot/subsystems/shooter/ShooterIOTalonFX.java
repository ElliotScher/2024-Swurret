package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftShooterTalon = new TalonFX(ShooterConstants.LEFT_DEVICE_ID);
  private final TalonFX rightShooterTalon = new TalonFX(ShooterConstants.RIGHT_DEVICE_ID);
  private final Alert leftDisconnectedAlert =
      new Alert("Shooter left Talon is disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert rightDisconnectedAlert =
      new Alert("Shooter right Talon is disconnected, check CAN bus.", AlertType.ERROR);

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

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    leftShooterTalon.getConfigurator().apply(config);
    rightShooterTalon.getConfigurator().apply(config);

    rightShooterTalon.setInverted(true);

    leftPosition = leftShooterTalon.getPosition();
    leftVelocity = leftShooterTalon.getVelocity();
    leftAppliedVolts = leftShooterTalon.getMotorVoltage();
    leftCurrent = leftShooterTalon.getSupplyCurrent();
    leftTemperature = leftShooterTalon.getDeviceTemp();

    rightPosition = rightShooterTalon.getPosition();
    rightVelocity = rightShooterTalon.getVelocity();
    rightAppliedVolts = rightShooterTalon.getMotorVoltage();
    rightCurrent = rightShooterTalon.getSupplyCurrent();
    rightTemperature = rightShooterTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, leftVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leftPosition, leftAppliedVolts, leftCurrent, leftTemperature);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, rightVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rightPosition, rightAppliedVolts, rightCurrent, rightTemperature);

    rightShooterTalon.optimizeBusUtilization();
    leftShooterTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean leftConnected =
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftCurrent, leftTemperature)
            .isOK();
    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPosition, rightVelocity, rightAppliedVolts, rightCurrent, rightTemperature)
            .isOK();
    leftDisconnectedAlert.set(!leftConnected);
    rightDisconnectedAlert.set(!rightConnected);

    inputs.leftPosition =
        Rotation2d.fromRotations(leftPosition.getValueAsDouble() * ShooterConstants.GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) * ShooterConstants.GEAR_RATIO;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = new double[] {leftCurrent.getValueAsDouble()};
    inputs.leftTempCelcius = new double[] {leftTemperature.getValueAsDouble()};

    inputs.rightPosition =
        Rotation2d.fromRotations(rightPosition.getValueAsDouble() * ShooterConstants.GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble()) * ShooterConstants.GEAR_RATIO;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = new double[] {rightCurrent.getValueAsDouble()};
    inputs.rightTempCelcius = new double[] {rightTemperature.getValueAsDouble()};
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftShooterTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightShooterTalon.setControl(new VoltageOut(volts));
  }
}

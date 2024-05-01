package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodTalon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelcius;

  private final Alert disconnecctedAlert =
      new Alert("Hood Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public HoodIOTalonFX() {
    hoodTalon = new TalonFX(HoodConstants.DEVICE_ID);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    hoodTalon.getConfigurator().apply(config);
    hoodTalon.setPosition(0.0);

    position = hoodTalon.getPosition();
    velocity = hoodTalon.getVelocity();
    appliedVolts = hoodTalon.getMotorVoltage();
    currentAmps = hoodTalon.getSupplyCurrent();
    tempCelcius = hoodTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, currentAmps, tempCelcius);
    hoodTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean isConnected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, tempCelcius)
            .isOK();
    disconnecctedAlert.set(!isConnected);

    inputs.position =
        Rotation2d.fromRotations(position.getValueAsDouble() / HoodConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble() / HoodConstants.GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {currentAmps.getValueAsDouble()};
    inputs.tempCelcius = new double[] {tempCelcius.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    hoodTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void resetPosition() {
    hoodTalon.setPosition(0.0);
  }
}

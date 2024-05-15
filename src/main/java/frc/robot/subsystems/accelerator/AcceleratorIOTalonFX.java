package frc.robot.subsystems.accelerator;

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

public class AcceleratorIOTalonFX implements AcceleratorIO {
  private final TalonFX acceleratorTalon = new TalonFX(AcceleratorConstants.DEVICE_ID);
  private final Alert disconnectedAlert =
      new Alert("Accelerator Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;
  private final StatusSignal<Double> temperature;

  public AcceleratorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = AcceleratorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    position = acceleratorTalon.getPosition();
    velocity = acceleratorTalon.getVelocity();
    appliedVolts = acceleratorTalon.getMotorVoltage();
    current = acceleratorTalon.getSupplyCurrent();
    temperature = acceleratorTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVolts, current, temperature);
    acceleratorTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AcceleratorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(velocity, position, appliedVolts, current, temperature).isOK();
    disconnectedAlert.set(!connected);

    inputs.position =
        Rotation2d.fromRotations(position.getValueAsDouble() / AcceleratorConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / AcceleratorConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelcius = temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    acceleratorTalon.setControl(new VoltageOut(volts));
  }
}

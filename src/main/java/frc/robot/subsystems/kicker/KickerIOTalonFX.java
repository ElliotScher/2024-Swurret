package frc.robot.subsystems.kicker;

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

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX kickerTalon = new TalonFX(KickerConstants.DEVICE_ID);
  private final Alert disconnectedAlert =
      new Alert("Kicker Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;
  private final StatusSignal<Double> temperature;

  public KickerIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = KickerConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    position = kickerTalon.getPosition();
    velocity = kickerTalon.getVelocity();
    appliedVolts = kickerTalon.getMotorVoltage();
    current = kickerTalon.getSupplyCurrent();
    temperature = kickerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVolts, current, temperature);
    kickerTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(velocity, position, appliedVolts, current, temperature).isOK();
    disconnectedAlert.set(!connected);

    inputs.position =
        Rotation2d.fromRotations(position.getValueAsDouble() / KickerConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / KickerConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
    inputs.tempCelcius = new double[] {temperature.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    kickerTalon.setControl(new VoltageOut(volts));
  }
}

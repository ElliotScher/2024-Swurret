package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX talon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;
  private final StatusSignal<Double> temperature;

  private final DigitalInput sensor;

  private final Alert talonDisconnectedAlert;

  public IntakeIOTalonFX() {
    talon = new TalonFX(IntakeConstants.DEVICE_ID);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    talon.getConfigurator().apply(motorConfig);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getSupplyCurrent();
    temperature = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current, temperature);
    talon.optimizeBusUtilization();

    sensor = new DigitalInput(IntakeConstants.SENSOR_CHANNEL);

    talonDisconnectedAlert =
        new Alert("Intake Talon is disconnected, check CAN bus", AlertType.ERROR);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean talonConnected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temperature).isOK();

    talonDisconnectedAlert.set(!talonConnected);
    inputs.position =
        Rotation2d.fromRotations(position.getValueAsDouble() / IntakeConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / IntakeConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelcius = temperature.getValueAsDouble();

    inputs.hasNote = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(new VoltageOut(volts));
  }
}

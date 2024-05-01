package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollersTalon;
  private final Solenoid solenoid;

  private final StatusSignal<Double> rollersPosition;
  private final StatusSignal<Double> rollersVelocity;
  private final StatusSignal<Double> rollersAppliedVolts;
  private final StatusSignal<Double> rollersCurrent;
  private final StatusSignal<Double> rollersTemperature;

  private final Alert rollersDisconnectedAlert =
      new Alert("Rollers Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public IntakeIOTalonFX() {
    rollersTalon = new TalonFX(IntakeConstants.DEVICE_ID);
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.SOLENOID_CHANNEL);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    rollersTalon.getConfigurator().apply(config);

    rollersPosition = rollersTalon.getPosition();
    rollersVelocity = rollersTalon.getVelocity();
    rollersAppliedVolts = rollersTalon.getMotorVoltage();
    rollersCurrent = rollersTalon.getSupplyCurrent();
    rollersTemperature = rollersTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, rollersVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollersPosition, rollersAppliedVolts, rollersCurrent, rollersTemperature);
    rollersTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean rollersConnected =
        BaseStatusSignal.refreshAll(
                rollersVelocity,
                rollersPosition,
                rollersAppliedVolts,
                rollersCurrent,
                rollersTemperature)
            .isOK();
    rollersDisconnectedAlert.set(!rollersConnected);

    inputs.rollersPositionRad =
        Units.rotationsToRadians(rollersPosition.getValueAsDouble()) / IntakeConstants.GEAR_RATIO;
    inputs.rollersVelocityRadPerSec =
        Units.rotationsToRadians(rollersVelocity.getValueAsDouble()) / IntakeConstants.GEAR_RATIO;
    inputs.rollersAppliedVolts = rollersAppliedVolts.getValueAsDouble();
    inputs.rollersCurrentAmps = new double[] {rollersCurrent.getValueAsDouble()};
    inputs.rollersTempCelcius = new double[] {rollersTemperature.getValueAsDouble()};

    inputs.leftPosition = solenoid.get();
  }

  @Override
  public void setRollersVoltage(double volts) {
    rollersTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setIntakePosition(boolean position) {
    solenoid.set(position);
  }

  @Override
  public void toggleIntakePosition() {
    solenoid.toggle();
  }
}

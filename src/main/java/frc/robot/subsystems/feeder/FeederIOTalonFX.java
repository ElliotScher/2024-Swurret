package frc.robot.subsystems.feeder;

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

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;

  private final StatusSignal<Double> leaderPosition;
  private final StatusSignal<Double> leaderVelocity;
  private final StatusSignal<Double> leaderAppliedVolts;
  private final StatusSignal<Double> leaderCurrent;
  private final StatusSignal<Double> leaderTemperature;

  private final StatusSignal<Double> followerPosition;
  private final StatusSignal<Double> followerVelocity;
  private final StatusSignal<Double> followerAppliedVolts;
  private final StatusSignal<Double> followerCurrent;
  private final StatusSignal<Double> followerTemperature;

  private final Alert leaderTalonDisconnectedAlert;
  private final Alert followerTalonDisconnectedAlert;

  public FeederIOTalonFX() {
    leaderTalon = new TalonFX(FeederConstants.LEADER_DEVICE_ID);
    followerTalon = new TalonFX(FeederConstants.FOLLOWER_DEVICE_ID);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    leaderTalon.getConfigurator().apply(motorConfig);
    followerTalon.getConfigurator().apply(motorConfig);

    followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), true));

    leaderPosition = leaderTalon.getPosition();
    leaderVelocity = leaderTalon.getVelocity();
    leaderAppliedVolts = leaderTalon.getMotorVoltage();
    leaderCurrent = leaderTalon.getSupplyCurrent();
    leaderTemperature = leaderTalon.getDeviceTemp();

    followerPosition = followerTalon.getPosition();
    followerVelocity = followerTalon.getVelocity();
    followerAppliedVolts = followerTalon.getMotorVoltage();
    followerCurrent = followerTalon.getSupplyCurrent();
    followerTemperature = followerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        leaderTemperature,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent,
        followerTemperature);
    leaderTalon.optimizeBusUtilization();
    followerTalon.optimizeBusUtilization();

    leaderTalonDisconnectedAlert =
        new Alert("Leader Feeder Talon is disconnected, check CAN bus", AlertType.ERROR);
    followerTalonDisconnectedAlert =
        new Alert("Follower Feeder Talon is disconnected, check CAN bus", AlertType.ERROR);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    boolean leaderTalonConnected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVolts,
                leaderCurrent,
                leaderTemperature)
            .isOK();
    boolean followerTalonConnected =
        BaseStatusSignal.refreshAll(
                followerPosition,
                followerVelocity,
                followerAppliedVolts,
                followerCurrent,
                followerTemperature)
            .isOK();

    leaderTalonDisconnectedAlert.set(!leaderTalonConnected);
    followerTalonDisconnectedAlert.set(!followerTalonConnected);

    inputs.leaderPosition =
        Rotation2d.fromRotations(leaderPosition.getValueAsDouble() / FeederConstants.GEAR_RATIO);
    inputs.leaderVelocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / FeederConstants.GEAR_RATIO;
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.leaderTempCelcius = leaderTemperature.getValueAsDouble();

    inputs.followerPosition =
        Rotation2d.fromRotations(followerPosition.getValueAsDouble() / FeederConstants.GEAR_RATIO);
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble()) / FeederConstants.GEAR_RATIO;
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.followerTempCelcius = followerTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leaderTalon.setControl(new VoltageOut(volts));
  }
}

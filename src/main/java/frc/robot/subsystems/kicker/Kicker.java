package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  @Getter private boolean isShooting = false;

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command shoot() {
    return runEnd(
        () -> {
          io.setVoltage(KickerConstants.SHOOT_VOLTAGE.get());
        },
        () -> stop());
  }

  public Command outtake() {
    return Commands.runEnd(
        () -> io.setVoltage(-KickerConstants.INTAKE_VOLTAGE.get()), () -> stop());
  }

  public Command runKicker() {
    return runEnd(
        () -> {
          io.setVoltage(KickerConstants.INTAKE_VOLTAGE.get());
          isShooting = true;
        },
        () -> {
          stop();
          isShooting = false;
        });
  }
}

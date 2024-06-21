package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final FeederIO io;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public boolean hasNote() {
    return inputs.hasNote;
  }

  public Command intake() {
    return Commands.run(() -> io.setVoltage(12.0)).until(() -> inputs.hasNote);
  }

  public Command eject() {
    return Commands.run(() -> io.setVoltage(-12.0));
  }

  public Command shoot() {
    return Commands.run(() -> io.setVoltage(12.0));
  }
}

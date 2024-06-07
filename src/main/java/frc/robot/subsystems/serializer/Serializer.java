package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();
  private final SerializerIO io;

  public Serializer(SerializerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command intake(BooleanSupplier feederHasNote) {
    return Commands.run(() -> io.setVoltage(12.0));
  }

  public Command eject() {
    return Commands.run(() -> io.setVoltage(-12.0));
  }

  public Command shoot() {
    return Commands.run(() -> io.setVoltage(12.0));
  }
}

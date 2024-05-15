package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();
  private static final DigitalInput sensor = new DigitalInput(SerializerConstants.SENSOR_CHANNEL);
  private final SerializerIO io;

  public Serializer(SerializerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Serializer", inputs);
    Logger.recordOutput("Note?", sensor.get());
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public boolean hasNote() {
    return sensor.get();
  }

  public Command shoot() {
    return runEnd(
        () -> {
          io.setVoltage(SerializerConstants.SHOOT_VOLTAGE.get());
        },
        () -> stop());
  }

  public Command intake() {
    return runEnd(() -> io.setVoltage(SerializerConstants.INTAKE_VOLTAGE.get()), () -> stop())
        .until(sensor::get);
  }

  public Command outtake() {
    return runEnd(() -> io.setVoltage(-SerializerConstants.INTAKE_VOLTAGE.get()), () -> stop());
  }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private final DigitalInput sensor;

  @Getter private boolean isIntaking;

  public Intake(IntakeIO io) {
    this.io = io;
    sensor = new DigitalInput(1);
    isIntaking = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command intake(BooleanSupplier feederHasNote) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  isIntaking = true;
                }),
            Commands.either(
                Commands.run(() -> io.setVoltage(12.0)).until(() -> sensor.get()),
                Commands.run(() -> io.setVoltage(12.0)),
                feederHasNote))
        .finallyDo(
            () -> {
              isIntaking = false;
            });
  }

  public Command eject() {
    return Commands.run(() -> io.setVoltage(-12.0));
  }

  public Command shoot() {
    return Commands.run(() -> io.setVoltage(12.0));
  }
}

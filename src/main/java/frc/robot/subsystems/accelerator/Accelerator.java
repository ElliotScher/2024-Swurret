package frc.robot.subsystems.accelerator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Accelerator extends SubsystemBase {
  private final AcceleratorIOInputsAutoLogged inputs = new AcceleratorIOInputsAutoLogged();

  private final AcceleratorIO io;

  public Accelerator(AcceleratorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Accelerator", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command runAccelerator() {
    return runEnd(
        () -> {
          io.setVoltage(AcceleratorConstants.SHOOT_VOLTAGE.get());
        },
        () -> stop());
  }
}

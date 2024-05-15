package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isDeployed() {
    return inputs.leftPosition && inputs.rightPosition;
  }

  private void stopRollers() {
    io.setRollersVoltage(0.0);
  }

  private void setIntakePosition(boolean position) {
    io.setIntakePosition(position);
  }

  public Command runVoltage() {
    return runEnd(
        () -> io.setRollersVoltage(IntakeConstants.ROLLERS_VOLTAGE.get()), () -> stopRollers());
  }

  public Command outtake() {
    return runEnd(() -> io.setRollersVoltage(-12), () -> stopRollers());
  }

  public Command deployIntake() {
    return runOnce(() -> setIntakePosition(true));
  }

  public Command retractIntake() {
    return runOnce(() -> setIntakePosition(false));
  }

  public Command singleActuation() {
    return startEnd(() -> setIntakePosition(true), () -> setIntakePosition(false));
  }
}

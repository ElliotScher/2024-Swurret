package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntegerWrapper {
  private int number;

  public IntegerWrapper(int number) {
    this.number = number;
  }

  public Command increment() {
    return Commands.runOnce(() -> number += 1);
  }

  public Command decrement() {
    return Commands.runOnce(() -> number -= 1);
  }

  public int getNumber() {
    return number;
  }
}

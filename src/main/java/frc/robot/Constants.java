// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final boolean TUNING_MODE = true;
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final RobotType ROBOT = RobotType.ROBOT_SIM_KRAKEN_X60_FOC;

  public static Mode getMode() {
    switch (ROBOT) {
      case ROBOT_TALONFX:
      case ROBOT_SPARK_FLEX:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIM_NEO:
      case ROBOT_SIM_VORTEX:
      case ROBOT_SIM_FALCON500:
      case ROBOT_SIM_FALCON500_FOC:
      case ROBOT_SIM_KRAKEN_X60:
      case ROBOT_SIM_KRAKEN_X60_FOC:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    ROBOT_SPARK_FLEX,
    ROBOT_TALONFX,
    ROBOT_SIM_VORTEX,
    ROBOT_SIM_NEO,
    ROBOT_SIM_FALCON500,
    ROBOT_SIM_FALCON500_FOC,
    ROBOT_SIM_KRAKEN_X60,
    ROBOT_SIM_KRAKEN_X60_FOC
  }

  /** Checks whether the robot the correct mode is selected when deploying. */
  public static void main(String... args) {
    if (ROBOT == RobotType.ROBOT_SIM_VORTEX) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}

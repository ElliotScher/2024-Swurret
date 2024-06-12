package frc.robot.util.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class SimulatorManager {

  public SimulatorManager() {
    new NoteShotSimulator(() -> new Pose3d(RobotState.getRobotPose()));
  }

  public static void periodic() {
    NoteShotSimulator.periodic();
  }

  public static Command manualShootNote(Turret turret, Hood hood, Shooter shooter) {
    return Commands.runOnce(
            () ->
                NoteShotSimulator.shootNote(
                    () -> turret.getFieldRelativePosition(RobotState.getRobotPose().getRotation()),
                    hood::getPosition,
                    shooter::getLeftSpeed,
                    shooter::getRightSpeed))
        .onlyIf(() -> RobotState.shooterReady());
  }
}

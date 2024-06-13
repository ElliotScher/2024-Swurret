package frc.robot.util.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteShotSimulator {
  private static final List<NoteState> notes = new ArrayList<NoteState>();

  private static Supplier<Pose3d> robotPoseSupplier;
  private static Pose3d[] notePoses;

  public NoteShotSimulator(Supplier<Pose3d> robotAngleSupplier) {
    NoteShotSimulator.robotPoseSupplier = robotAngleSupplier;
  }

  public static void periodic() {
    for (int i = 0; i < notes.size(); i++) {
      notes.get(i).updateNoteState();
      if (noteScored(notes.get(i))) {
        notes.remove(i);
      }
    }
    notePoses = new Pose3d[notes.size()];
    for (int i = 0; i < notes.size(); i++) {
      notePoses[i] = notes.get(i).getNotePose();
    }

    Logger.recordOutput("Physics Simulator/Notes", notePoses);
  }

  public static void shootNote(
      Supplier<Rotation2d> turretPosition,
      Supplier<Rotation2d> hoodPosition,
      DoubleSupplier leftShooterSpeed,
      DoubleSupplier rightShooterSpeed) {

    double leftFlywheelLinearSpeed =
        leftShooterSpeed.getAsDouble() * (ShooterConstants.WHEEL_DIAMETER / 2.0);
    double rightFlywheelLinearSpeed =
        rightShooterSpeed.getAsDouble() * (ShooterConstants.WHEEL_DIAMETER / 2.0);

    double noteSpeed = Math.abs((leftFlywheelLinearSpeed + rightFlywheelLinearSpeed) / 2.0);
    double angularSpeed =
        Math.abs(
            (leftFlywheelLinearSpeed - rightFlywheelLinearSpeed)
                / (2.0 * NoteConstants.NOTE_RADIUS));

    double noteLinearVelocityX = turretPosition.get().getCos() * noteSpeed;
    double noteLinearVelocityY = turretPosition.get().getSin() * noteSpeed;
    double noteLinearVelocityZ = hoodPosition.get().getSin() * noteSpeed;

    double angularVelocityYaw = 0.0;
    if (leftShooterSpeed.getAsDouble() > rightShooterSpeed.getAsDouble()) {
      angularVelocityYaw = -angularSpeed;
    } else {
      angularVelocityYaw = angularSpeed;
    }
    notes.add(
        new NoteState(
            robotPoseSupplier
                .get()
                .transformBy(
                    new Transform3d(
                        new Translation3d(0.0, 0.0, ShooterConstants.FLOOR_TO_HOOD_PIVOT),
                        new Rotation3d(
                            0.0,
                            hoodPosition.get().getRadians(),
                            turretPosition.get().getRadians()))),
            noteLinearVelocityX,
            noteLinearVelocityY,
            noteLinearVelocityZ,
            0.0,
            0.0,
            -9.81,
            angularVelocityYaw,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0));
  }

  private static boolean noteScored(NoteState note) {
    // create bounding box for speaker and check if note pose is in that box
    return false;
  }
}

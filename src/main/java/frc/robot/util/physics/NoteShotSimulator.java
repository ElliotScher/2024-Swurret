package frc.robot.util.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteShotSimulator {
  private static final List<NoteState> notes = new ArrayList<NoteState>();

  private static Supplier<Pose3d> robotPoseSupplier;

  public NoteShotSimulator(Supplier<Pose3d> robotAngleSupplier) {
    NoteShotSimulator.robotPoseSupplier = robotAngleSupplier;
  }

  public static void periodic() {
    Pose3d[] notePoses = new Pose3d[notes.size()];
    for (int i = 0; i < notes.size(); i++) {
      notePoses[i] = notes.get(i).getNotePose();
    }
    for (int i = 0; i < notes.size(); i++) {
      notes.get(i).updateNoteState();
      if (noteScored(notes.get(i))) {
        notes.remove(i);
      }
    }

    Logger.recordOutput("Physics Simulator/Notes", notePoses);
  }

  public static void shootNote(
      Supplier<Rotation2d> turretPosition,
      Supplier<Rotation2d> hoodPosition,
      DoubleSupplier leftShooterSpeed,
      DoubleSupplier rightShooterSpeed) {
    // need to do physics on how all this works
    notes.add(
        new NoteState(
            robotPoseSupplier.get().transformBy(ShooterConstants.ROBOT_TO_SHOOTER_TRANSFORM),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
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

package frc.robot.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import lombok.Getter;
import lombok.Setter;

public class NoteState {
  @Getter @Setter private Pose3d notePose;
  private double noteLinearVelocityX;
  private double noteLinearVelocityY;
  private double noteLinearVelocityZ;
  private double noteLinearAccelerationX;
  private double noteLinearAccelerationY;
  private double noteLinearAccelerationZ;
  private double noteAngularVelocityYaw;
  private double noteAngularVelocityPitch;
  private double noteAngularVelocityRoll;
  private double noteAngularAccelerationYaw;
  private double noteAngularAccelerationPitch;
  private double noteAngularAccelerationRoll;

  public NoteState(Pose3d notePose) {
    this.notePose = notePose;
    this.noteLinearVelocityX = 0.0;
    this.noteLinearVelocityY = 0.0;
    this.noteLinearVelocityZ = 0.0;
    this.noteLinearAccelerationX = 0.0;
    this.noteLinearAccelerationY = 0.0;
    this.noteLinearAccelerationZ = 0.0;
    this.noteAngularVelocityYaw = 0.0;
    this.noteAngularVelocityPitch = 0.0;
    this.noteAngularVelocityRoll = 0.0;
    this.noteAngularAccelerationYaw = 0.0;
    this.noteAngularAccelerationPitch = 0.0;
    this.noteAngularAccelerationRoll = 0.0;
  }

  public NoteState(
      Pose3d notePose,
      double noteLinearVelocityX,
      double noteLinearVelocityY,
      double noteLinearVelocityZ,
      double noteLinearAccelerationX,
      double noteLinearAccelerationY,
      double noteLinearAccelerationZ,
      double noteAngularVelocityYaw,
      double noteAngularVelocityPitch,
      double noteAngularVelocityRoll,
      double noteAngularAccelerationYaw,
      double noteAngularAccelerationPitch,
      double noteAngularAccelerationRoll) {
    this.notePose = notePose;
    this.noteLinearVelocityX = noteLinearVelocityX;
    this.noteLinearVelocityY = noteLinearVelocityY;
    this.noteLinearVelocityZ = noteLinearVelocityZ;
    this.noteLinearAccelerationX = noteLinearAccelerationX;
    this.noteLinearAccelerationY = noteLinearAccelerationY;
    this.noteLinearAccelerationZ = noteLinearAccelerationZ;
    this.noteAngularVelocityYaw = noteAngularVelocityYaw;
    this.noteAngularVelocityPitch = noteAngularVelocityPitch;
    this.noteAngularVelocityRoll = noteAngularVelocityRoll;
    this.noteAngularAccelerationYaw = noteAngularAccelerationYaw;
    this.noteAngularAccelerationPitch = noteAngularAccelerationPitch;
    this.noteAngularAccelerationRoll = noteAngularAccelerationRoll;
  }

  public void applyForce(LinearAxis axis, double force) {
    switch (axis) {
      case LINEAR_X:
        noteLinearAccelerationX += force / NoteConstants.NOTE_MASS_KG;
        break;
      case LINEAR_Y:
        noteLinearAccelerationY += force / NoteConstants.NOTE_MASS_KG;
        break;
      case LINEAR_Z:
        noteLinearAccelerationZ += force / NoteConstants.NOTE_MASS_KG;
        break;
    }
  }

  public void clearForce(LinearAxis axis) {
    switch (axis) {
      case LINEAR_X:
        noteLinearAccelerationX = 0.0;
        break;
      case LINEAR_Y:
        noteLinearAccelerationY = 0.0;
        break;
      case LINEAR_Z:
        noteLinearAccelerationZ = 0.0;
        break;
    }
  }

  public void clearForces() {
    noteLinearAccelerationX = 0.0;
    noteLinearAccelerationY = 0.0;
    noteLinearAccelerationZ = 0.0;
  }

  public void applyMoment(AngularAxis axis, double moment) {
    switch (axis) {
      case ANGULAR_YAW:
        noteAngularAccelerationYaw += moment / NoteConstants.NOTE_MOI_YAW_KGM2;
        break;
      case ANGULAR_PITCH:
        noteAngularAccelerationPitch += moment / NoteConstants.NOTE_MOI_PITCH_KGM2;
        break;
      case ANGULAR_ROLL:
        noteAngularAccelerationRoll += moment / NoteConstants.NOTE_MOI_ROLL_KGM2;
        break;
    }
  }

  public void clearMoment(AngularAxis axis) {
    switch (axis) {
      case ANGULAR_YAW:
        noteAngularAccelerationYaw = 0.0;
        break;
      case ANGULAR_PITCH:
        noteAngularAccelerationPitch = 0.0;
        break;
      case ANGULAR_ROLL:
        noteAngularAccelerationRoll = 0.0;
        break;
    }
  }

  public void clearMoments() {
    noteAngularAccelerationYaw = 0.0;
    noteAngularAccelerationPitch = 0.0;
    noteAngularAccelerationRoll = 0.0;
  }

  public void updateNoteState() {
    if (notePose.getTranslation().getZ() > 0.0) {
      noteLinearVelocityX += noteLinearAccelerationX * Constants.LOOP_PERIOD_SECS;
      noteLinearVelocityY += noteLinearAccelerationY * Constants.LOOP_PERIOD_SECS;
      noteLinearVelocityZ += noteLinearAccelerationZ * Constants.LOOP_PERIOD_SECS;
      noteAngularVelocityYaw += noteAngularAccelerationYaw * Constants.LOOP_PERIOD_SECS;
      noteAngularVelocityPitch += noteAngularAccelerationPitch * Constants.LOOP_PERIOD_SECS;
      noteAngularVelocityRoll += noteAngularAccelerationRoll * Constants.LOOP_PERIOD_SECS;
      notePose =
          notePose.transformBy(
              new Transform3d(
                  new Translation3d(
                      noteLinearVelocityX * Constants.LOOP_PERIOD_SECS,
                      noteLinearVelocityY * Constants.LOOP_PERIOD_SECS,
                      noteLinearVelocityZ * Constants.LOOP_PERIOD_SECS),
                  new Rotation3d(
                      noteAngularVelocityRoll * Constants.LOOP_PERIOD_SECS,
                      noteAngularVelocityPitch * Constants.LOOP_PERIOD_SECS,
                      noteAngularVelocityYaw * Constants.LOOP_PERIOD_SECS)));
    } else {
      notePose = new Pose3d(notePose.getX(), notePose.getY(), 0.0, new Rotation3d());
    }
  }
}

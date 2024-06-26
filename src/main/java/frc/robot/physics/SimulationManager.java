package frc.robot.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.geometry.Box2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {
  private static final List<NoteState> notes = new ArrayList<NoteState>();
  private static Pose3d[] notePoses;
  private static Box2d intakeBox;
  private static boolean isIntaking;
  private static boolean hasNote;

  public SimulationManager() {
    for (int i = 0; i < 3; i++) {
      notes.add(
          new NoteState(
              new Pose3d(
                  new Pose2d(
                      FieldConstants.StagingLocations.spikeTranslations[i], new Rotation2d()))));
    }
    for (int i = 0; i < 3; i++) {
      notes.add(
          new NoteState(
              new Pose3d(
                  new Pose2d(
                      new Translation2d(
                          FieldConstants.fieldLength
                              - FieldConstants.StagingLocations.spikeTranslations[i].getX(),
                          FieldConstants.StagingLocations.spikeTranslations[i].getY()),
                      new Rotation2d()))));
    }
    for (int i = 0; i < 5; i++) {
      notes.add(
          new NoteState(
              new Pose3d(
                  new Pose2d(
                      FieldConstants.StagingLocations.centerlineTranslations[i],
                      new Rotation2d()))));
    }
    intakeBox =
        new Box2d(
            RobotState.getRobotPose()
                .transformBy(IntakeConstants.CENTER_TO_INTAKE_OUTER_EDGE_TRANSFORM),
            IntakeConstants.LENGTH,
            IntakeConstants.WIDTH);
  }

  public static void periodic() {
    intakeBox =
        new Box2d(
            RobotState.getRobotPose()
                .transformBy(IntakeConstants.CENTER_TO_INTAKE_OUTER_EDGE_TRANSFORM),
            IntakeConstants.LENGTH,
            IntakeConstants.WIDTH);
    for (int i = 0; i < notes.size(); i++) {
      notes.get(i).updateNoteState();
      if (isIntaking
          && !hasNote
          && intakeBox
                  .closestPoint(notes.get(i).getNotePose().toPose2d())
                  .getDistance(notes.get(i).getNotePose().toPose2d().getTranslation())
              <= NoteConstants.NOTE_RADIUS) {
        notes.remove(i);
        hasNote = true;
        break;
      }
      if (noteScored(notes.get(i))) {
        notes.remove(i);
        break;
      }
    }
    notePoses = new Pose3d[notes.size()];
    for (int i = 0; i < notes.size(); i++) {
      notePoses[i] = notes.get(i).getNotePose();
    }

    Logger.recordOutput("Physics Simulator/Notes", notePoses);
    Logger.recordOutput("Physics Simulator/Intake Box2d", intakeBox.getCorners());
    Logger.recordOutput("Physics Simulator/Note?", hasNote);
    Logger.recordOutput("Physics Simulator/Intaking?", isIntaking);
  }

  public static Command shootNote(
      Supplier<Rotation2d> turretPosition,
      Supplier<Rotation2d> hoodPosition,
      DoubleSupplier leftShooterSpeed,
      DoubleSupplier rightShooterSpeed,
      BooleanSupplier shooterReady) {
    return Commands.runOnce(
            () -> {
              double leftFlywheelLinearSpeed =
                  leftShooterSpeed.getAsDouble() * (ShooterConstants.WHEEL_DIAMETER / 2.0);
              double rightFlywheelLinearSpeed =
                  rightShooterSpeed.getAsDouble() * (ShooterConstants.WHEEL_DIAMETER / 2.0);

              double noteSpeed =
                  Math.abs((leftFlywheelLinearSpeed + rightFlywheelLinearSpeed) / 2.0);

              double noteLinearVelocityX = noteSpeed;
              double noteLinearVelocityY = 0.0;
              double noteLinearVelocityZ = 0.0;

              if (hasNote) {
                notes.add(
                    new NoteState(
                        new Pose3d(RobotState.getRobotPose())
                            .transformBy(
                                new Transform3d(
                                    new Translation3d(
                                        ShooterConstants.CENTER_TO_SHOOTER_PIVOT,
                                        0.0,
                                        ShooterConstants.FLOOR_TO_HOOD_PIVOT),
                                    new Rotation3d(
                                        0.0,
                                        hoodPosition
                                            .get()
                                            .plus(Rotation2d.fromDegrees(-90))
                                            .getRadians(),
                                        turretPosition.get().getRadians()))),
                        noteLinearVelocityX,
                        noteLinearVelocityY,
                        noteLinearVelocityZ,
                        0.0,
                        0.0,
                        -9.81,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0));
              }
              hasNote = false;
            })
        .onlyIf(shooterReady)
        .repeatedly();
  }

  public static Command manualShootNote(
      Supplier<Rotation2d> turretPosition,
      Supplier<Rotation2d> hoodPosition,
      DoubleSupplier leftShooterSpeed,
      DoubleSupplier rightShooterSpeed) {
    return Commands.runOnce(
        () -> {
          double leftFlywheelLinearSpeed =
              leftShooterSpeed.getAsDouble() * (ShooterConstants.WHEEL_DIAMETER / 2.0);
          double rightFlywheelLinearSpeed =
              rightShooterSpeed.getAsDouble() * (ShooterConstants.WHEEL_DIAMETER / 2.0);

          double noteSpeed = Math.abs((leftFlywheelLinearSpeed + rightFlywheelLinearSpeed) / 2.0);

          double noteLinearVelocityX = noteSpeed;
          double noteLinearVelocityY = 0.0;
          double noteLinearVelocityZ = 0.0;

          notes.add(
              new NoteState(
                  new Pose3d(RobotState.getRobotPose())
                      .transformBy(
                          new Transform3d(
                              new Translation3d(
                                  ShooterConstants.CENTER_TO_SHOOTER_PIVOT,
                                  0.0,
                                  ShooterConstants.FLOOR_TO_HOOD_PIVOT),
                              new Rotation3d(
                                  0.0,
                                  hoodPosition.get().plus(Rotation2d.fromDegrees(-90)).getRadians(),
                                  turretPosition.get().getRadians()))),
                  noteLinearVelocityX,
                  noteLinearVelocityY,
                  noteLinearVelocityZ,
                  0.0,
                  0.0,
                  -9.81,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0));
        });
  }

  public static Command clearNotes() {
    return Commands.runOnce(notes::clear);
  }

  public static Command intake() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  if (!isIntaking) {
                    isIntaking = true;
                  }
                })
            .finallyDo(
                () -> {
                  isIntaking = false;
                }));
  }

  private static boolean noteScored(NoteState note) {
    return note.getNotePose().getX() >= FieldConstants.fieldLength
        || note.getNotePose().getX() <= 0.0;
  }
}

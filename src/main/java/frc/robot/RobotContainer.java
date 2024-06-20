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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.physics.SimulationManager;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOSparkFlex;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSparkFlex;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkFlex;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerIOSim;
import frc.robot.subsystems.serializer.SerializerIOSparkFlex;
import frc.robot.subsystems.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkFlex;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight3G;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Mechanism3d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Intake intake;
  private Serializer serializer;
  private Turret turret;
  private Feeder feeder;
  private Hood hood;
  private Shooter shooter;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard Inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case ROBOT_SPARK_FLEX:
          // Test robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSparkFlex(0),
                  new ModuleIOSparkFlex(1),
                  new ModuleIOSparkFlex(2),
                  new ModuleIOSparkFlex(3));
          intake = new Intake(new IntakeIOSparkFlex());
          serializer = new Serializer(new SerializerIOSparkFlex());
          turret = new Turret(new TurretIOSparkFlex());
          feeder = new Feeder(new FeederIOSparkFlex());
          hood = new Hood(new HoodIOSparkFlex());
          shooter = new Shooter(new ShooterIOSparkFlex());
          vision =
              new Vision(
                  new CameraIOLimelight3G(0),
                  new CameraIOLimelight3G(1),
                  new CameraIOLimelight3G(2),
                  new CameraIOLimelight3G(3));
          break;
        case ROBOT_TALONFX:
          // Snapback, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          intake = new Intake(new IntakeIOTalonFX());
          serializer = new Serializer(new SerializerIOTalonFX());
          turret = new Turret(new TurretIOTalonFX());
          feeder = new Feeder(new FeederIOTalonFX());
          hood = new Hood(new HoodIOTalonFX());
          shooter = new Shooter(new ShooterIOTalonFX());
          vision =
              new Vision(
                  new CameraIOLimelight3G(0),
                  new CameraIOLimelight3G(1),
                  new CameraIOLimelight3G(2),
                  new CameraIOLimelight3G(3));
          break;
        case ROBOT_SIM_NEO:
        case ROBOT_SIM_VORTEX:
        case ROBOT_SIM_FALCON500:
        case ROBOT_SIM_FALCON500_FOC:
        case ROBOT_SIM_KRAKEN_X60:
        case ROBOT_SIM_KRAKEN_X60_FOC:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          intake = new Intake(new IntakeIOSim());
          serializer = new Serializer(new SerializerIOSim());
          turret = new Turret(new TurretIOSim());
          feeder = new Feeder(new FeederIOSim());
          hood = new Hood(new HoodIOSim());
          shooter = new Shooter(new ShooterIOSim());
          vision =
              new Vision(
                  new CameraIOSim(0), new CameraIOSim(1), new CameraIOSim(2), new CameraIOSim(3));
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision =
          new Vision(new CameraIO() {}, new CameraIO() {}, new CameraIO() {}, new CameraIO() {});
    }

    // Configure autobuilder.
    AutoBuilder.configureHolonomic(
        RobotState::getRobotPose,
        RobotState::resetRobotPose,
        () -> DriveConstants.KINEMATICS.toChassisSpeeds(drive.getModuleStates()),
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.MAX_LINEAR_VELOCITY,
            DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "LocalADStarAK/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("LocalADStarAK/Trajectory Setpoint", targetPose);
        });

    // Configure auto choices.
    autoChooser = new LoggedDashboardChooser<>("Auto Routines");
    autoChooser.addDefaultOption("None", AutoRoutines.none());

    // Configure RobotState
    new RobotState(
        drive::getRotation,
        drive::getFieldRelativeVelocity,
        drive::getModulePositions,
        vision::getCameraTypes,
        vision::getPrimaryVisionPoses,
        vision::getSecondaryVisionPoses,
        vision::getPrimaryPoseTimestamps,
        vision::getSecondaryPoseTimestamps,
        turret::atGoal,
        hood::atGoal,
        shooter::atGoal,
        intake::isIntaking);

    // Configure the button bindings
    configureButtonBindings();

    if (!(Constants.ROBOT.equals(Constants.RobotType.ROBOT_TALONFX)
        || Constants.ROBOT.equals(Constants.RobotType.ROBOT_SPARK_FLEX))) {
      new SimulationManager();
      configureSimulationButtonBindings();
    }

    // Configure shuffleboard
    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Hood Offset", RobotState::getSpeakerAngleCompensation)
        .withPosition(0, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Flywheel Offset", RobotState::getSpeakerFlywheelCompensation)
        .withPosition(0, 1)
        .withSize(1, 1);
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
    driver.leftBumper().whileTrue(CompositeCommands.getCollectCommand(intake, serializer, feeder));
    driver
        .rightBumper()
        .whileTrue(
            CompositeCommands.getShootSpeakerCommand(
                intake, serializer, turret, feeder, hood, shooter));
    driver
        .leftTrigger()
        .whileTrue(
            CompositeCommands.getShootAmpCommand(
                intake, serializer, turret, feeder, hood, shooter));
    driver
        .rightTrigger()
        .whileTrue(
            CompositeCommands.getFeedCommand(intake, serializer, turret, feeder, hood, shooter));
  }

  private void configureSimulationButtonBindings() {
    driver
        .a()
        .whileTrue(
            CompositeCommands.getShootSpeakerCommand(
                    intake, serializer, turret, feeder, hood, shooter)
                .alongWith(
                    SimulationManager.shootNote(
                        turret::getPosition,
                        hood::getPosition,
                        shooter::getLeftSpeed,
                        shooter::getRightSpeed)));
    driver
        .b()
        .onTrue(
            SimulationManager.manualShootNote(
                turret::getPosition,
                hood::getPosition,
                shooter::getLeftSpeed,
                shooter::getRightSpeed));
    driver.x().onTrue(SimulationManager.clearNotes());
  }

  public void updateMechanism3d() {
    Logger.recordOutput(
        "Mechanism3d", Mechanism3d.getPoses(turret.getPosition(), hood.getPosition()));
  }

  public Command getAutonomousCommand() {
    return AutoRoutines.center_abc(drive, intake, serializer, turret, feeder, hood, shooter);
  }
}

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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight3;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard Inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case SNAPBACK:
          // Snapback, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          vision =
              new Vision(
                  new CameraIOLimelight3(0),
                  new CameraIOLimelight3(1),
                  new CameraIOLimelight3(2),
                  new CameraIOLimelight3(3));
          break;
        case ROBOT_2K24_TEST:
          // Test robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          break;

        case ROBOT_SIM:
          // Sim robot, instantiate physics sim IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          break;
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

    // Configure autobuilder
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
        vision::getSecondaryPoseTimestamps);

    // Configure the button bindings
    configureButtonBindings();

    // Configure shuffleboard
    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Hood Offset", RobotState::getHoodOffset)
        .withPosition(0, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Flywheel Offset", RobotState::getFlywheelOffset)
        .withPosition(0, 1)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .add("Shooter View", CameraServer.addSwitchedCamera("limelight-shooter").getSource())
        .withPosition(1, 0)
        .withSize(5, 5)
        .withWidget("Camera Stream");
    Shuffleboard.getTab("Teleoperated")
        .add("Intake View", CameraServer.addSwitchedCamera("limelight-intake").getSource())
        .withPosition(6, 0)
        .withSize(5, 5)
        .withWidget("Camera Stream");
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            vision,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper()));
    driver.start().onTrue(CompositeCommands.resetHeading());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}

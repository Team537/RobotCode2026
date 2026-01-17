// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.swerve.CompositeDriveCommand;
import frc.robot.commands.swerve.DriveToSequenceCommand;
import frc.robot.commands.swerve.ManualRotationVelocityDirective;
import frc.robot.commands.swerve.ManualTranslationVelocityDirective;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.EnumPrettifier;
import frc.robot.util.field.Alliance;
import frc.robot.util.swerve.requests.RotationDirective;
import frc.robot.util.swerve.requests.TranslationDirective;


public class RobotContainer {

  XboxController controller = new XboxController(0);

  DriveSubsystem driveSubsystem;

  private final SendableChooser<Alliance> allianceSelector = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
  }

  public void setupSmartDashboard() {

    // Alliance
    EnumPrettifier.setupSendableChooserFromEnum(allianceSelector, Alliance.class, Alliance.RED);
    SmartDashboard.putData("Alliance",allianceSelector);

    // Pose Rest
    Command resetPoseCommand = new InstantCommand(() -> {
        Pose2d pose =
            allianceSelector.getSelected() == Alliance.BLUE
                ? Constants.Drive.BLUE_STARTING_POSE
                : Constants.Drive.RED_STARTING_POSE;

        driveSubsystem.setPose(pose);
    }).ignoringDisable(true);

    SmartDashboard.putData("Reset Pose", resetPoseCommand);

  }

  

  public void scheduleTeleOp() {

    // Setup the translational directive for drive subsystem
    TranslationDirective manualTranslationVelocityDirective = new ManualTranslationVelocityDirective(
      driveSubsystem,
      () -> -controller.getLeftY(), // X (On controllers, -Y corresponds to "forwards", or +X)
      () -> -controller.getLeftX(), // X (On controllers, -X corresponds to "left", or +Y)
      () -> controller.getRightTriggerAxis(), 
      () -> controller.getLeftTriggerAxis(), 
      () -> !controller.getLeftBumperButton(), 
      Constants.Operator.Drive.TRANSLATION_INPUT_CURVE_POWER, 
      Constants.Operator.Drive.NORMAL_TRANSLATION_MAX_SPEED, 
      Constants.Operator.Drive.THROTTLE_TRANSLATION_MAX_SPEED, 
      Constants.Operator.Drive.SLOW_TRANSLATION_MAX_SPEED, 
      allianceSelector.getSelected().driverRotation
    );

    // Setup the rotational directive for drive subsystem
    RotationDirective manualRotationVelocityDirective = new ManualRotationVelocityDirective(
      driveSubsystem,
      () -> -controller.getRightX(), // X (On controllers, -X corresponds to "counter-clockwise", or +Theta)
      () -> controller.getRightTriggerAxis(), 
      () -> controller.getLeftTriggerAxis(), 
      Constants.Operator.Drive.ROTATION_INPUT_CURVE_POWER, 
      Constants.Operator.Drive.NORMAL_ROTATION_MAX_SPEED, 
      Constants.Operator.Drive.THROTTLE_ROTATION_MAX_SPEED, 
      Constants.Operator.Drive.SLOW_ROTATION_MAX_SPEED
    );

    Command manualDriveCommand = new CompositeDriveCommand(driveSubsystem, manualTranslationVelocityDirective, manualRotationVelocityDirective, null, null);
    driveSubsystem.setDefaultCommand(manualDriveCommand);

  }

  public void scheduleAutonomous() {
    
    // Example: Drive to multiple coordinates in sequence
    // The robot will drive to coordinate 1, then coordinate 2, then coordinate 3
    // Far new Pose2d(1.46, 2.91, Rotation2d.fromDegrees(0))
    // Close new Pose2d(2.63, 3.51, Rotation2d.fromDegrees(0))
    new DriveToSequenceCommand(
      driveSubsystem,
      new Pose2d(1.46, 2.91, Rotation2d.fromDegrees(0)),
      new Pose2d(2.63, 3.51, Rotation2d.fromDegrees(0))
    ).schedule();

    // Alternative builder pattern syntax:
    // new DriveToSequenceCommand.Builder(driveSubsystem)
    //   .addPose(2.0, 6.0, 0)      // x, y, rotation in degrees
    //   .addPose(4.0, 6.0, 90)
    //   .addPose(4.0, 4.0, 180)
    //   .build()
    //   .schedule();

  }

}

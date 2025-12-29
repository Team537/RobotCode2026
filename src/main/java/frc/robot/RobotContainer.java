// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.CompositeDriveCommand;
import frc.robot.commands.swerve.ManualRotationVelocityDirective;
import frc.robot.commands.swerve.ManualTranslationPositionDirective;
import frc.robot.commands.swerve.ManualTranslationVelocityDirective;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.EnumPrettifier;
import frc.robot.util.field.Alliance;
import frc.robot.util.swerve.requests.TranslationDirective;
import frc.robot.util.swerve.requests.RotationDirective;


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
    SmartDashboard.putData(allianceSelector);

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
      Rotation2d.kCCW_Pi_2
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

    TranslationDirective manualTranslationPositionDirective = new ManualTranslationPositionDirective(
      driveSubsystem, 
      () -> -controller.getLeftY(), // X (On controllers, -Y corresponds to "forwards", or +X)
      () -> -controller.getLeftX(), 
      Constants.Operator.Drive.TARGET_TRANSLATION_RADIUS, 
      Rotation2d.kCCW_Pi_2
    );

    RotationDirective manualRotationDirectiv

    Command manualDriveCommand = new CompositeDriveCommand(driveSubsystem, manualTranslationVelocityDirective, manualRotationVelocityDirective, null, null);
    driveSubsystem.setDefaultCommand(manualDriveCommand);

    Trigger targetTranslationTrigger = new Trigger(() -> controller.getLeftStickButton());


  }

  public void scheduleAutonomous() {
    
    driveSubsystem.setPose(new Pose2d(2.0,1.0,Rotation2d.kPi));
    driveSubsystem.getVelocityCommand(new ChassisSpeeds(5.0, 0, 12.0)).schedule();
    /*driveSubsystem.getPathfindToPoseCommand(new Pose2d(11.0,4.0,Rotation2d.kPi))
    .andThen(driveSubsystem.getPathfindToPoseCommand(new Pose2d(2.0,4.0,Rotation2d.kZero)))
    .andThen(driveSubsystem.getStopCommand())
    .schedule();*/

  }

}

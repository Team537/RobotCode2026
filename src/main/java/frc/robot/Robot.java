// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandTimeline;
import frc.robot.util.field.FieldStatePublisher;

public class Robot extends TimedRobot {

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
    robotContainer.setupSmartDashboard();
  }

  @Override
  public void robotInit() {
    DeployMetadata.publishAtStartup();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    FieldStatePublisher.update();
    CommandTimeline.run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandTimeline.cancelAll();
    FieldStatePublisher.setupElasticNotifications();
    robotContainer.scheduleAutonomous();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.scheduleTeleOp();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

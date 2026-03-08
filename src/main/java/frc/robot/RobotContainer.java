// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.zip.ZipException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeAndShootWhileDriving;
import frc.robot.commands.ShootPreloadCommand;
import frc.robot.commands.swerve.CompositeDriveCommand;
import frc.robot.commands.swerve.DriveToSequenceCommand;
import frc.robot.commands.swerve.ManualRotationVelocityDirective;
import frc.robot.commands.swerve.ManualTranslationVelocityDirective;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.Raycast;
import frc.robot.util.EnumPrettifier;
import frc.robot.util.auto.IntakeStrategy;
import frc.robot.util.dashboard.AdjustableDouble;
import frc.robot.util.field.Alliance;
import frc.robot.util.field.FieldUtil;
import frc.robot.util.swerve.SwerveUtil;
import frc.robot.util.swerve.requests.RotationDirective;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationDirective;
import frc.robot.util.swerve.requests.TranslationRequest;
import frc.robot.util.turret.TurretSolver;
import frc.robot.util.vision.detections.RobotDetection;

public class RobotContainer {

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  DriveSubsystem driveSubsystem;
  TurretSubsystem turretSubsystem;
  ShooterSubsystem shooterSubsystem;
  TransferSubsystem transferSubsystem;
  IntakePivotSubsystem intakePivot;
  IntakeRollerSubsystem intakeRoller;
  Raycast raycast;

  private final Field2d targetingField = new Field2d();
  private final Field2d robotField = new Field2d();

  private static final String[] FIXED_TARGETS = { "A", "B" };
  private static final String[] ROBOT_TARGETS = { "X", "Y" };

  AdjustableDouble turretOffsetDegrees;
  AdjustableDouble hoodOffsetDegrees;
  AdjustableDouble shooterPercent;

  private static enum FixedTarget {
    A,
    B
  }

  FixedTarget selectedFixedTarget = FixedTarget.A;
  boolean xHeld = false;
  boolean yHeld = false;

  Supplier<Translation3d> targetingSupplier = () -> Translation3d.kZero;

  SendableChooser<IntakeStrategy> intakeStrategyChooser = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    intakePivot = new IntakePivotSubsystem();
    intakeRoller = new IntakeRollerSubsystem();
    turretSubsystem = new TurretSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    transferSubsystem = new TransferSubsystem();

    shooterSubsystem.setYawPitchSuppliers(() -> turretSubsystem.getAngle(), () -> turretSubsystem.getHoodAngle());

    setupSmartDashboard();
    configureBindings();

    // Setup Raycast.
    raycast = Raycast.getInstance();
    raycast.start();
  }

  public void setupSmartDashboard() {

    // Pose Reset
    Command resetPoseCommand = new InstantCommand(() -> {
      Pose2d pose = FieldUtil.getAlliance()
          .map(alliance -> switch (alliance) {
            case BLUE -> Constants.Drive.BLUE_STARTING_POSE;
            case RED -> Constants.Drive.RED_STARTING_POSE;
          })
          .orElse(Constants.Drive.BLUE_STARTING_POSE);

      // apply the pose
      driveSubsystem.setPose(pose);
    }).ignoringDisable(true);

    SmartDashboard.putData("Reset Pose", resetPoseCommand);

    Command resetTurretCommand = new InstantCommand(() -> {
      turretSubsystem.resetTurretAngle(Constants.Turret.START_POSITION);
    }).ignoringDisable(true);

    Command resetHoodCommand = new InstantCommand(() -> {
      turretSubsystem.resetHoodAngle(Constants.Turret.HOOD_START_POSITION);
    }).ignoringDisable(true);

    SmartDashboard.putData("Reset Turret Angle", resetTurretCommand);
    SmartDashboard.putData("Reset Hood Angle", resetTurretCommand);
    // Targeting Data

    // Fixed field targets (static presets with forced defaults every boot)
    for (String key : FIXED_TARGETS) {
      String path = "Targeting/FixedTargets/" + key + "/";

      Translation3d defaultTranslation;

      switch (key) {
        case "A":
          defaultTranslation = Constants.Field.BLUE_DEPO_TRANSLATION;
          break;

        case "B":
          defaultTranslation = Constants.Field.BLUE_OUTPOST_TRANSLATION;
          break;

        default:
          defaultTranslation = new Translation3d();
          break;
      }

      SmartDashboard.putNumber(path + "X", defaultTranslation.getX());
      SmartDashboard.putNumber(path + "Y", defaultTranslation.getY());
      SmartDashboard.putNumber(path + "Z", defaultTranslation.getZ());
    }

    // Robot-tracking targets
    for (String key : ROBOT_TARGETS) {
      String path = "Targeting/RobotTargets/" + key + "/";
      SmartDashboard.putString(path + "TeamNumber", "");
      SmartDashboard.putNumber(path + "TargetHeight", 0.25);
      SmartDashboard.putBoolean(path + "UseFallback", false);
      SmartDashboard.putNumber(path + "FallbackX", 0.0);
      SmartDashboard.putNumber(path + "FallbackY", 0.0);
    }

    // Set the targetting field's robot pose far away so its off screen
    targetingField.setRobotPose(new Pose2d(100.0, 100.0, Rotation2d.kZero));
    SmartDashboard.putData("Targeting/Field", targetingField);

    SmartDashboard.putData("Robot/Field", robotField);

    // Operator Adjustments

    turretOffsetDegrees = new AdjustableDouble("ErrorSettings/TurretOffset", 0.0, Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY);
    turretOffsetDegrees.setDashboardRounding(Constants.Operator.ErrorSettings.TURRET_OFFSET_DECIMAL_PLACE);

    hoodOffsetDegrees = new AdjustableDouble("ErrorSettings/HoodOffset", 0.0, Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY);
    hoodOffsetDegrees.setDashboardRounding(Constants.Operator.ErrorSettings.HOOD_OFFSET_DECIMAL_PLACE);

    shooterPercent = new AdjustableDouble("ErrorSettings/ShooterPercent",
        Constants.Operator.ErrorSettings.SHOOTER_PERCENT_DEFAULT, Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY);
    shooterPercent.setDashboardRounding(Constants.Operator.ErrorSettings.SHOOTER_PERCENT_DECIMAL_PLACE);

    turretSubsystem.setTurretOffsetSupplier(
        () -> Rotation2d.fromDegrees(-turretOffsetDegrees.get()));
    turretSubsystem.setHoodOffsetSupplier(
        () -> Rotation2d.fromDegrees(hoodOffsetDegrees.get()));
    shooterSubsystem
        .setSpeedMultiplierSupplier(() -> shooterPercent.get() / 100.0);

    SmartDashboard.putNumber("Auto/StartDelay", Constants.Operator.Auto.DEFAULT_START_DELAY);
    SmartDashboard.putNumber("Auto/PreloadShootTime", Constants.Operator.Auto.DEFAULT_PRELOAD_SHOOT_TIME);
    SmartDashboard.putNumber("Auto/IntakeShootTime", Constants.Operator.Auto.DEFAULT_INTAKE_SHOOT_TIME);

    SmartDashboard.putNumber("Auto/CustomReadyPose/X", 0.0);
    SmartDashboard.putNumber("Auto/CustomReadyPose/Y", 0.0);
    SmartDashboard.putNumber("Auto/CustomReadyPose/Theta", 0.0);
    SmartDashboard.putNumber("Auto/CustomIntakePose/X", 0.0);
    SmartDashboard.putNumber("Auto/CustomIntakePose/Y", 0.0);
    SmartDashboard.putNumber("Auto/CustomIntakePose/Theta", 0.0);
    SmartDashboard.putNumber("Auto/CustomTargetTranslation/X", 0.0);
    SmartDashboard.putNumber("Auto/CustomTargetTranslation/Y", 0.0);
    SmartDashboard.putNumber("Auto/CustomTargetTranslation/Z", 0.0);

    SmartDashboard.putBoolean("Auto/RunAuto", true);

    EnumPrettifier.setupSendableChooserFromEnum(intakeStrategyChooser, IntakeStrategy.class, IntakeStrategy.JUST_SHOOT);
    SmartDashboard.putData("Auto/IntakeStrategy", intakeStrategyChooser);

  }

  /**
   * Updates all targeting objects on the Field2d display.
   * Automatically mirrors fixed targets if on Red alliance.
   */
  public void updateFieldObjects() {

    // Update Fixed Targets
    for (String key : FIXED_TARGETS) {

      String path = "Targeting/FixedTargets/" + key + "/";

      double x = SmartDashboard.getNumber(path + "X", 0.0);
      double y = SmartDashboard.getNumber(path + "Y", 0.0);

      targetingField
          .getObject("Fixed " + key)
          .setPose(new Pose2d(x, y, new Rotation2d()));

    }

    // Update Robot Targets (using fallback translations only for display)
    for (String key : ROBOT_TARGETS) {

      String path = "Targeting/RobotTargets/" + key + "/";

      double x = SmartDashboard.getNumber(path + "FallbackX", 0.0);
      double y = SmartDashboard.getNumber(path + "FallbackY", 0.0);

      targetingField
          .getObject("Robot " + key)
          .setPose(new Pose2d(x, y, new Rotation2d()));
    }

    robotField.setRobotPose(driveSubsystem.getPose());
    robotField.getObject("Target").setPose(new Pose2d(targetingSupplier.get().toTranslation2d(), Rotation2d.kZero));

  }

  public void configureBindings() {

    // Driver controls

    Trigger stowTrigger = new Trigger(
        () -> driverController.getBButton() || SwerveUtil.willRobotEnterRegion(driveSubsystem.getPose(),
            driveSubsystem.getVelocity(), Constants.Field.TRENCH_REGION, Constants.Drive.HOOD_STOW_LOOKAHEAD_TIME));
    stowTrigger.and(() -> !FieldUtil.isAutonomous()).whileTrue(
        turretSubsystem.getStowCommand());

    Trigger shootTrigger = new Trigger(() -> driverController.getRightBumperButton());

    Trigger intakeTrigger = new Trigger(() -> driverController.getAButton());

    Trigger solverValid = new Trigger(() -> TurretSolver.solve(driveSubsystem.getPose(), driveSubsystem.getVelocity(),
        targetingSupplier.get(), Constants.Turret.SOLVER_CONFIG).isValid());

    solverValid.onTrue(
        Commands.runOnce(() -> SmartDashboard.putBoolean("Turret/SolverValid", true)).ignoringDisable(true)).onFalse(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Turret/SolverValid", false)).ignoringDisable(true));

    /* Shooter runs while button held */
    shootTrigger.whileTrue(
        shooterSubsystem.getTargetCommand(
            targetingSupplier,
            driveSubsystem::getPose,
            driveSubsystem::getVelocity));

    /* Transfer runs ONLY while button AND solver valid */
    shootTrigger
        .whileTrue(
            transferSubsystem.getLoadCommand());

    /* Intake pivot runs while button held */
    intakeTrigger.whileTrue(
        intakePivot.deployIntakeCommand());

    /* Intake roller runs while button held */
    intakeTrigger.whileTrue(
        intakeRoller.getIntakeCommand());

    /* Stop shooter on button release */
    shootTrigger.onFalse(
        Commands.parallel(
            transferSubsystem.getStopCommand(),
            shooterSubsystem.getStopCommand()));

    /* Stop intake on button release */
    intakeTrigger.onFalse(
        Commands.parallel(
            intakePivot.raiseIntakeCommand(),
            intakeRoller.getStopCommand()));

    // ==============================
    // Turret Offset Adjustment (POV Left / Right)
    // ==============================

    // POV Left (225°–315°) : Decrease turret offset
    new Trigger(() -> {
      int pov = operatorController.getPOV();
      return pov >= 225 && pov <= 315;
    })
        .whileTrue(
            turretOffsetDegrees.getHeldIntervalCommand(-Constants.Operator.ErrorSettings.TURRET_OFFSET_INCREASE,
                Constants.Operator.ErrorSettings.SETTINGS_DELAY_TIME));

    // POV Right (45°–135°) : Increase turret offset
    new Trigger(() -> {
      int pov = operatorController.getPOV();
      return pov >= 45 && pov <= 135;
    })
        .whileTrue(
            turretOffsetDegrees.getHeldIntervalCommand(Constants.Operator.ErrorSettings.TURRET_OFFSET_INCREASE,
                Constants.Operator.ErrorSettings.SETTINGS_DELAY_TIME));

    // ==============================
    // Hood Offset Adjustment (D-Pad Up / D-Pad Down)
    // ==============================

    // D-Pad Up : Increase hood offset
    new Trigger(() -> {
      int pov = operatorController.getPOV();
      return pov >= 315 || (pov >= 0 && pov <= 45);
    })
        .whileTrue(
            hoodOffsetDegrees.getHeldIntervalCommand(Constants.Operator.ErrorSettings.HOOD_OFFSET_INCREASE,
                Constants.Operator.ErrorSettings.SETTINGS_DELAY_TIME));

    // D-Pad Down : Decrease hood offset
    new Trigger(() -> {
      int pov = operatorController.getPOV();
      return pov >= 135 && pov <= 225;
    })
        .whileTrue(
            hoodOffsetDegrees.getHeldIntervalCommand(-Constants.Operator.ErrorSettings.HOOD_OFFSET_INCREASE,
                Constants.Operator.ErrorSettings.SETTINGS_DELAY_TIME));

    // ==============================
    // Shooter Percent Adjustment (Left Bumper / Right Bumper)
    // ==============================

    // Left Bumper : Decrease shooter percent
    new Trigger(() -> {
      return operatorController.getLeftBumperButton();
    })
        .whileTrue(
            shooterPercent.getHeldIntervalCommand(-Constants.Operator.ErrorSettings.SHOOTER_PERCENT_INCREASE,
                Constants.Operator.ErrorSettings.SETTINGS_DELAY_TIME));

    // Right Bumper : Increase shooter percent
    new Trigger(() -> {
      return operatorController.getRightBumperButton();
    })
        .whileTrue(
            shooterPercent.getHeldIntervalCommand(Constants.Operator.ErrorSettings.SHOOTER_PERCENT_INCREASE,
                Constants.Operator.ErrorSettings.SETTINGS_DELAY_TIME));

    new Trigger(
        () -> operatorController.getAButton()).onTrue(
            new InstantCommand(() -> selectedFixedTarget = FixedTarget.A));

    new Trigger(
        () -> operatorController.getBButton()).onTrue(
            new InstantCommand(() -> selectedFixedTarget = FixedTarget.B));

    new Trigger(
        () -> operatorController.getXButton()).onTrue(
            new InstantCommand(() -> xHeld = true))
        .onFalse(
            new InstantCommand(() -> xHeld = false));

    new Trigger(
        () -> operatorController.getYButton()).onTrue(
            new InstantCommand(() -> yHeld = true))
        .onFalse(
            new InstantCommand(() -> yHeld = false));

    targetingSupplier = () -> {
      Translation2d robotPosition = driveSubsystem.getPose().getTranslation();

      // 1 - Alliance hub targeting
      Optional<Alliance> alliance = FieldUtil.getAlliance();
      if (alliance.isPresent()) {
        if (FieldUtil.flipIfRed(Constants.Field.BLUE_ALLIANCE_ZONE).contains(robotPosition)) {
          return FieldUtil.flipIfRed(Constants.Field.BLUE_HUB_TRANSLATION);
        }
      }

      // 2 - Vision robot targeting
      if (xHeld || yHeld) {
        String targetKey = xHeld ? "X" : "Y";
        String basePath = "Targeting/RobotTargets/" + targetKey + "/";

        // Parse team number from string safely
        int teamNumber;
        try {
          teamNumber = Integer.parseInt(SmartDashboard.getString(basePath + "TeamNumber", "-1"));
        } catch (NumberFormatException e) {
          teamNumber = -1;
        }

        // Get the robot using the alliance if possible. We will never want to target a
        // robot of the opposing alliance.
        Optional<RobotDetection> detectedRobot;
        if (alliance.isPresent()) {
          detectedRobot = raycast.getRobot(teamNumber, alliance.get(), 1);
        } else {
          detectedRobot = raycast.getRobot(teamNumber, 1);
        }

        if (teamNumber > 0) {
          detectedRobot = Optional.empty();
        }

        // Fallback info
        boolean useFallback = SmartDashboard.getBoolean(basePath + "UseFallback", false);
        double fallbackX = SmartDashboard.getNumber(basePath + "FallbackX", 0.0);
        double fallbackY = SmartDashboard.getNumber(basePath + "FallbackY", 0.0);
        double targetHeight = SmartDashboard.getNumber(basePath + "TargetHeight", 0.25);

        if (detectedRobot.isPresent()) {
          return detectedRobot.get().getPoseTranslation3d();
        } else if (useFallback) {
          Translation3d fallbackTarget = new Translation3d(fallbackX, fallbackY, targetHeight);
          return FieldUtil.flipIfRed(fallbackTarget);
        }
      }

      // 3 - Fixed target fallback (A/B)
      switch (selectedFixedTarget) {

        case A: {
          String basePath = "Targeting/FixedTargets/A/";

          double x = SmartDashboard.getNumber(basePath + "X", 0.0);
          double y = SmartDashboard.getNumber(basePath + "Y", 0.0);
          double z = SmartDashboard.getNumber(basePath + "Z", 0.0);
          return FieldUtil.flipIfRed(new Translation3d(x, y, z));
        }

        case B: {
          String basePath = "Targeting/FixedTargets/B/";

          double x = SmartDashboard.getNumber(basePath + "X", 0.0);
          double y = SmartDashboard.getNumber(basePath + "Y", 0.0);
          double z = SmartDashboard.getNumber(basePath + "Z", 0.0);

          return FieldUtil.flipIfRed(new Translation3d(x, y, z));
        }

        default: {
          // Default safely to A if somehow null
          String basePath = "Targeting/FixedTargets/A/";

          double x = SmartDashboard.getNumber(basePath + "X", 0.0);
          double y = SmartDashboard.getNumber(basePath + "Y", 0.0);
          double z = SmartDashboard.getNumber(basePath + "Z", 0.0);

          return FieldUtil.flipIfRed(new Translation3d(x, y, z));
        }
      }

    };

  }

  public void scheduleTeleOp() {

    // Setup the translational directive for drive subsystem
    TranslationDirective manualTranslationVelocityDirective = new ManualTranslationVelocityDirective(
        driveSubsystem,
        () -> -driverController.getLeftY(), // X (On controllers, -Y corresponds to "forwards", or +X)
        () -> -driverController.getLeftX(), // X (On controllers, -X corresponds to "left", or +Y)
        () -> driverController.getRightTriggerAxis(),
        () -> driverController.getLeftTriggerAxis(),
        () -> !driverController.getLeftBumperButton(),
        Constants.Operator.Drive.TRANSLATION_INPUT_CURVE_POWER,
        Constants.Operator.Drive.NORMAL_TRANSLATION_MAX_SPEED,
        Constants.Operator.Drive.THROTTLE_TRANSLATION_MAX_SPEED,
        Constants.Operator.Drive.SLOW_TRANSLATION_MAX_SPEED,
        FieldUtil.getAlliance().orElse(Alliance.BLUE).driverRotation,
        Rotation2d.kPi);

    // Setup the rotational directive for drive subsystem
    RotationDirective manualRotationVelocityDirective = new ManualRotationVelocityDirective(
        driveSubsystem,
        () -> -driverController.getRightX(), // X (On controllers, -X corresponds to "counter-clockwise", or +Theta)
        () -> driverController.getRightTriggerAxis(),
        () -> driverController.getLeftTriggerAxis(),
        Constants.Operator.Drive.ROTATION_INPUT_CURVE_POWER,
        Constants.Operator.Drive.NORMAL_ROTATION_MAX_SPEED,
        Constants.Operator.Drive.THROTTLE_ROTATION_MAX_SPEED,
        Constants.Operator.Drive.SLOW_ROTATION_MAX_SPEED);

    // Scheduling Subsystems
    Command manualDriveCommand = new CompositeDriveCommand(driveSubsystem, manualTranslationVelocityDirective,
        manualRotationVelocityDirective, null, null);
    driveSubsystem.setDefaultCommand(manualDriveCommand);

    turretSubsystem.setDefaultCommand(turretSubsystem.getTargetCommand(
        targetingSupplier,
        driveSubsystem::getPose,
        driveSubsystem::getVelocity));

  }

  public void scheduleAutonomous() {

    Command auto = Commands.sequence(

        Commands.deadline(
            Commands
                .waitSeconds(SmartDashboard.getNumber("Auto/StartDelay", Constants.Operator.Auto.DEFAULT_START_DELAY)),
            intakePivot.raiseIntakeCommand(),
            turretSubsystem.getTargetCommand(() -> FieldUtil.flipIfRed(Constants.Field.BLUE_HUB_TRANSLATION),
                driveSubsystem::getPose, driveSubsystem::getVelocity)),

        new ShootPreloadCommand(
            shooterSubsystem,
            turretSubsystem,
            transferSubsystem,
            () -> FieldUtil.flipIfRed(Constants.Field.BLUE_HUB_TRANSLATION),
            driveSubsystem::getPose,
            driveSubsystem::getVelocity,
            SmartDashboard.getNumber("Auto/PreloadShootTime", Constants.Operator.Auto.DEFAULT_PRELOAD_SHOOT_TIME)),

        getIntakeStrategyCommand()

    );

    if (SmartDashboard.getBoolean("Auto/RunAuto", true)) {
      CommandScheduler.getInstance().schedule(auto);
    }

  }

  private Command getIntakeStrategyCommand() {

    IntakeStrategy strategy = intakeStrategyChooser.getSelected();

    switch (strategy) {

      case JUST_SHOOT:
        return Commands.none();

      case DEPOT:
        return new IntakeAndShootWhileDriving(
            driveSubsystem,
            intakePivot,
            intakeRoller,
            shooterSubsystem,
            turretSubsystem,
            transferSubsystem,
            () -> FieldUtil.flipIfRed(Constants.Field.BLUE_HUB_TRANSLATION),
            Constants.Operator.Auto.DEPOT_READY_INTAKE_POSE,
            Constants.Operator.Auto.DEPOT_INTAKE_POSE,
            false,
            Constants.Operator.Auto.AUTO_INTAKE_MAX_SPEED,
            SmartDashboard.getNumber("Auto/IntakeShootTime", Constants.Operator.Auto.DEFAULT_INTAKE_SHOOT_TIME));

      case OUTPOST:
        return new IntakeAndShootWhileDriving(
            driveSubsystem,
            intakePivot,
            intakeRoller,
            shooterSubsystem,
            turretSubsystem,
            transferSubsystem,
            () -> FieldUtil.flipIfRed(Constants.Field.BLUE_HUB_TRANSLATION),
            Constants.Operator.Auto.OUTPOST_READY_INTAKE_POSE,
            Constants.Operator.Auto.OUTPOST_INTAKE_POSE,
            false,
            Constants.Operator.Auto.AUTO_INTAKE_MAX_SPEED,
            SmartDashboard.getNumber("Auto/IntakeShootTime", Constants.Operator.Auto.DEFAULT_INTAKE_SHOOT_TIME));

      case NEUTRAL_LEFT:
        return Commands.sequence(

            new DriveToSequenceCommand(
                driveSubsystem,
                Constants.Operator.Auto.NEUTRAL_LEFT_SEQUENCE_ONE.stream()
                    .map(FieldUtil::flipIfRed)
                    .toList()),

            new RunCommand(
                () -> driveSubsystem.driveWithCompositeRequests(
                    new TranslationRequest.Position(
                        FieldUtil.flipIfRed(Constants.Operator.Auto.NEUTRAL_LEFT_SPIN_TRANSLATION)),
                    new RotationRequest.Velocity(-Constants.Operator.Auto.NEUTRAL_SPIN_SPEED)),
                driveSubsystem).withTimeout(Constants.Operator.Auto.NEUTRAL_SPIN_TIME),

            new DriveToSequenceCommand(
                driveSubsystem,
                Constants.Operator.Auto.NEUTRAL_LEFT_SEQUENCE_TWO.stream()
                    .map(FieldUtil::flipIfRed)
                    .toList()));

      case NEUTRAL_RIGHT:
        return Commands.sequence(

            new DriveToSequenceCommand(
                driveSubsystem,
                Constants.Operator.Auto.NEUTRAL_RIGHT_SEQUENCE_ONE.stream()
                    .map(FieldUtil::flipIfRed)
                    .toList()),

            new RunCommand(
                () -> driveSubsystem.driveWithCompositeRequests(
                    new TranslationRequest.Position(
                        FieldUtil.flipIfRed(Constants.Operator.Auto.NEUTRAL_RIGHT_SPIN_TRANSLATION)),
                    new RotationRequest.Velocity(Constants.Operator.Auto.NEUTRAL_SPIN_SPEED)),
                driveSubsystem).withTimeout(Constants.Operator.Auto.NEUTRAL_SPIN_TIME),

            new DriveToSequenceCommand(
                driveSubsystem,
                Constants.Operator.Auto.NEUTRAL_RIGHT_SEQUENCE_TWO.stream()
                    .map(FieldUtil::flipIfRed)
                    .toList()));

      case RAM_LEFT:
        return new DriveToSequenceCommand(driveSubsystem, Constants.Operator.Auto.RAM_LEFT_SEQUENCE.stream()
            .map(FieldUtil::flipIfRed)
            .toList());

      case RAM_RIGHT:
        return new DriveToSequenceCommand(driveSubsystem, Constants.Operator.Auto.RAM_RIGHT_SEQUENCE.stream()
            .map(FieldUtil::flipIfRed)
            .toList());

      case CUSTOM:

        Pose2d ready = getDashboardPose("Auto/CustomReadyPose");
        Pose2d intake = getDashboardPose("Auto/CustomIntakePose");
        Translation3d target = getDashboardTranslation3d("Auto/CustomTargetTranslation");

        boolean stow = SmartDashboard.getBoolean("Auto/CustomStowTurret", true);

        return new IntakeAndShootWhileDriving(
            driveSubsystem,
            intakePivot,
            intakeRoller,
            shooterSubsystem,
            turretSubsystem,
            transferSubsystem,
            () -> FieldUtil.flipIfRed(target),
            FieldUtil.flipIfRed(ready),
            FieldUtil.flipIfRed(intake),
            stow,
            Constants.Operator.Auto.AUTO_INTAKE_MAX_SPEED,
            SmartDashboard.getNumber("Auto/IntakeShootTime", Constants.Operator.Auto.DEFAULT_INTAKE_SHOOT_TIME));

      default:
        return Commands.none();
    }
  }

  private Pose2d getDashboardPose(String prefix) {
    double x = SmartDashboard.getNumber(prefix + "/X", 0.0);
    double y = SmartDashboard.getNumber(prefix + "/Y", 0.0);
    double rotDeg = SmartDashboard.getNumber(prefix + "/Theta", 0.0);

    return new Pose2d(
        x,
        y,
        Rotation2d.fromDegrees(rotDeg));

  }

  private Translation3d getDashboardTranslation3d(String prefix) {
    double x = SmartDashboard.getNumber(prefix + "/X", 0.0);
    double y = SmartDashboard.getNumber(prefix + "/Y", 0.0);
    double z = SmartDashboard.getNumber(prefix + "/Z", 0.0);

    return new Translation3d(
        x,
        y,
        z);

  }

}

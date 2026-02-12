package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.PhotonVisionOdometry;
import frc.robot.subsystems.vision.Raycast;
import frc.robot.util.swerve.Obstacle;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationRequest;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    private SwerveDrive swerveDrive;
    private PhotonVisionOdometry visionOdometry;
    private Raycast raycast;

    PIDController xController;
    PIDController yController;
    ProfiledPIDController thetaController;
    HolonomicDriveController driveController;

    private double translationalTolerance = Constants.Drive.TRANSLATIONAL_TOLERANCE;
    private Rotation2d rotationalTolerance = Constants.Drive.ROTATIONAL_TOLERANCE;

    private List<Supplier<List<Obstacle>>> obstaclesSuppliers;
    
    // Feature Flags 
    private boolean useVisionOdometry = true;

    public DriveSubsystem() {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
            swerveDrive = new SwerveParser(Constants.Drive.YAGSL_CONFIG).createSwerveDrive(Constants.Drive.MAX_TRANSLATIONAL_SPEED, Pose2d.kZero);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setMaximumAllowableSpeeds(Constants.Drive.MAX_TRANSLATIONAL_SPEED, Constants.Drive.MAX_ROTATIONAL_SPEED);
        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(true);
        swerveDrive.setAngularVelocityCompensation(true, true, Constants.Drive.ANGULAR_VELOCITY_COMPENSATION_COEFFICIENT);
        swerveDrive.setModuleEncoderAutoSynchronize(true,Constants.Drive.ENCODER_AUTO_SYNCHRONIZE_DEADBAND.getDegrees());
        swerveDrive.zeroGyro();

        RobotConfig pathPlannerConfig;
        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeVelocity,
                (speeds, feedForwards) -> {
                    driveWithVelocityRobotRelative(
                        ChassisSpeeds.discretize(speeds,0.02)
                    );
                },
                new PPHolonomicDriveController(
                    new PIDConstants(Constants.Drive.TRANSLATIONAL_KP,Constants.Drive.TRANSLATIONAL_KI,Constants.Drive.TRANSLATIONAL_KD), 
                    new PIDConstants(Constants.Drive.ROTATIONAL_KP,Constants.Drive.ROTATIONAL_KI,Constants.Drive.ROTATIONAL_KD)
                ),
                pathPlannerConfig,
                () -> false,
                this
            );

        } catch (Exception e) {
            e.printStackTrace();
        }

        xController = new PIDController(Constants.Drive.TRANSLATIONAL_KP,Constants.Drive.TRANSLATIONAL_KI,Constants.Drive.TRANSLATIONAL_KD);
        yController = new PIDController(Constants.Drive.TRANSLATIONAL_KP,Constants.Drive.TRANSLATIONAL_KI,Constants.Drive.TRANSLATIONAL_KD);
        thetaController = new ProfiledPIDController(Constants.Drive.ROTATIONAL_KP,Constants.Drive.ROTATIONAL_KI,Constants.Drive.ROTATIONAL_KD,new Constraints(Constants.Drive.MAX_ROTATIONAL_SPEED, Constants.Drive.MAX_ROTATIONAL_ACCELERATION));
        thetaController.enableContinuousInput(-Math.PI,Math.PI);

        obstaclesSuppliers = new ArrayList<>();

        PathfindingCommand.warmupCommand().schedule();

        SmartDashboard.putNumber("Compensation Coefficient", Constants.Drive.ANGULAR_VELOCITY_COMPENSATION_COEFFICIENT);

        // Setup vision odometry (if enabled).
        if (useVisionOdometry) {
            visionOdometry = new PhotonVisionOdometry(
                () -> swerveDrive.getPose(),
                swerveDrive.field
            );
        }

        // Grab a reference to the Raycast singleton.
        raycast = Raycast.getInstance();
    }

    // ------------------------------
    // PERIODIC
    // ------------------------------

    /**
     * Periodically updates the pathfinding system with dynamic obstacles.
     *
     * <p>This method collects all obstacles from registered suppliers, converts them
     * into PathPlanner-friendly format (bounding boxes), and updates the Pathfinding
     * system with the current robot position.
     */
    @Override
    public void periodic() {
        // Flatten all obstacle lists from suppliers
        List<Obstacle> obstacles = new ArrayList<>();
        obstaclesSuppliers.forEach(supplier -> supplier.get().forEach(obstacles::add));

        // Convert obstacles to PathPlanner bounding boxes (bottom-left + top-right)
        List<Pair<Translation2d, Translation2d>> pathPlannerObstacles = new ArrayList<>();
        obstacles.forEach(obstacle -> pathPlannerObstacles.add(obstacle.toPathPlannerObstacle()));

        // Update the Pathfinding system with obstacles and robot current translation
        Pathfinding.setDynamicObstacles(pathPlannerObstacles, getPose().getTranslation());

        swerveDrive.setAngularVelocityCompensation(true, true, SmartDashboard.getNumber("Compensation Coefficient", Constants.Drive.ANGULAR_VELOCITY_COMPENSATION_COEFFICIENT));

        // Update vision odometry if enabled.
        if (useVisionOdometry) {
            swerveDrive.updateOdometry(); // Vision being enabled requires manual updates to odometry.
            visionOdometry.updatePoseEstimation(swerveDrive);
          }

        // Update Raycast's pose.
        Pose2d pose = getPose();
        raycast.publishRobotPose(pose);

        // Publish data to the dashboard for visualization.
        SmartDashboard.putNumber("X Position", pose.getX());
        SmartDashboard.putNumber("Y Position", pose.getY());
        SmartDashboard.putNumber("Theta Position", pose.getRotation().getDegrees());

    }

    /**
     * Adds a supplier of obstacles to be included in the pathfinding system.
     *
     * <p>The supplier should return a List of obstacles each time it is called.
     *
     * @param supplier Supplier of dynamic obstacles.
     */
    public void addObstaclesSupplier(Supplier<List<Obstacle>> supplier) {
        obstaclesSuppliers.add(supplier);
    }

    /**
     * Removes a previously added obstacle supplier.
     *
     * @param supplier Supplier to remove.
     */
    public void removeObstaclesSupplier(Supplier<List<Obstacle>> supplier) {
        obstaclesSuppliers.remove(supplier);
    }

    // ------------------------------
    // POSE ACCESS
    // ------------------------------

    /**
     * Returns the current estimated robot pose on the field.
     * Synchronized to ensure thread safety.
     * 
     * @return The robot's pose as a {@link Pose2d}.
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the robot's odometry to a given pose.
     *
     * <p>This should be used at initialization or after an external correction
     * (e.g., vision measurement or field reset).
     *
     * @param pose The pose to reset the odometry to.
     */
    public void setPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    // ------------------------------
    // Drive Method
    // ------------------------------

    /**
     * Drives the robot using composite translation and rotation requests.
     * <p>
     * This method supports any combination of translation and rotation directives:
     * <ul>
     *   <li>{@link TranslationRequest.Velocity}/{@link RotationRequest.Velocity}: translational or rotational velocity. Translation velocities
     *       are field-oriented by default unless {@code fieldRelative} is false.</li>
     *   <li>{@link TranslationRequest.Position}/{@link RotationRequest.Position}: target position
     *       for the subsystem. Uses the respective controllers to calculate the necessary velocities.</li>
     *   <li>{@link TranslationRequest.Stop}/{@link RotationRequest.Stop}: stops motion in the respective DOF.</li>
     * </ul>
     * <p>
     * If a request is {@code null}, it will default to a stopped request.
     *
     * @param tReq the translation request (velocity, position, or stop)
     * @param rReq the rotation request (velocity, position, or stop)
     */
    public void driveWithCompositeRequests(TranslationRequest tReq, RotationRequest rReq) {
        // Default to stopped requests if null
        if (tReq == null) tReq = new TranslationRequest.Stop();
        if (rReq == null) rReq = new RotationRequest.Stop();

        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;

        // --- Handle translation ---
        if (tReq instanceof TranslationRequest.Velocity vel) {
            vx = vel.velocity().getX();
            vy = vel.velocity().getY();
            // fieldRelative is handled below in final drive call
        } else if (tReq instanceof TranslationRequest.Position pos) {
            vx = xController.calculate(getPose().getX(), pos.position().getX());
            vy = yController.calculate(getPose().getY(), pos.position().getY());
        } else if (tReq instanceof TranslationRequest.Stop) {
            vx = 0.0;
            vy = 0.0;
        }

        // --- Handle rotation ---
        if (rReq instanceof RotationRequest.Velocity rotVel) {
            omega = rotVel.velocity();
        } else if (rReq instanceof RotationRequest.Position rotPos) {
            omega = thetaController.calculate(getPose().getRotation().getRadians(),
                                            rotPos.position().getRadians());
        } else if (rReq instanceof RotationRequest.Stop) {
            omega = 0.0;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx,vy,omega);

        // Clamp chassis speeds to max velocities
        double currentSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        if (currentSpeed > Constants.Drive.MAX_TRANSLATIONAL_SPEED) {
            double scale = Constants.Drive.MAX_TRANSLATIONAL_SPEED / currentSpeed;
            chassisSpeeds.vxMetersPerSecond *= scale;
            chassisSpeeds.vyMetersPerSecond *= scale;
        }
        
        // Clamp rotational speed
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > Constants.Drive.MAX_ROTATIONAL_SPEED) {
            chassisSpeeds.omegaRadiansPerSecond = Math.copySign(
                Constants.Drive.MAX_ROTATIONAL_SPEED, 
                chassisSpeeds.omegaRadiansPerSecond
            );
        }

        // --- Drive: field-oriented unless robot-relative velocity ---
        if (tReq instanceof TranslationRequest.Velocity vel && !vel.fieldRelative()) {
            swerveDrive.drive(chassisSpeeds); // robot-relative velocity
        } else {
            swerveDrive.driveFieldOriented(chassisSpeeds); // default field-oriented
        }
    }

    public void driveWithVelocity(ChassisSpeeds velocity) {
        driveWithCompositeRequests(
            new TranslationRequest.Velocity(
                new Translation2d(
                    velocity.vxMetersPerSecond,
                    velocity.vyMetersPerSecond
                ),
                true
            ),
            new RotationRequest.Velocity(
                velocity.omegaRadiansPerSecond
            )
        );
    }

    public void driveWithVelocityRobotRelative(ChassisSpeeds velocity) {
        driveWithCompositeRequests(
            new TranslationRequest.Velocity(
                new Translation2d(
                    velocity.vxMetersPerSecond,
                    velocity.vyMetersPerSecond
                ),
                false
            ),
            new RotationRequest.Velocity(
                velocity.omegaRadiansPerSecond
            )
        );
    }

    public void driveToPosition(Pose2d pose) {
        driveWithCompositeRequests(
            new TranslationRequest.Position(
                pose.getTranslation()
            ),
            new RotationRequest.Position(
                pose.getRotation()
            )
        );
    }

    public void stop() {
        driveWithCompositeRequests(
            new TranslationRequest.Stop(),
            new RotationRequest.Stop()
        );
    }

    // ------------------------------
    // VELOCITY ACCESS
    // ------------------------------

    /**
     * Returns the robot's current velocity relative to its own coordinate frame.
     *
     * @return Robot-relative chassis velocity (vx, vy, omega) in m/s and rad/s.
     */
    public ChassisSpeeds getRobotRelativeVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Returns the robot's current velocity in field coordinates.
     *
     * @return Field-relative chassis velocity (vx, vy, omega) in m/s and rad/s.
     */
    public ChassisSpeeds getVelocity() {
        return swerveDrive.getFieldVelocity();
    }
    
    // ------------------------------
    // VELOCITY COMMANDS
    // ------------------------------

    /**
     * Returns a command that drives the swerve using the supplied field-relative chassis speeds.
     *
     * @param speeds Supplier that provides the desired chassis speeds each loop.
     * @return A RunCommand that drives the swerve field-relative.
     */
    public Command getVelocityCommand(Supplier<ChassisSpeeds> speeds) {
        return run(() -> driveWithVelocity(speeds.get()));
    }

    /**
     * Returns a command that drives the swerve using the supplied robot-relative chassis speeds.
     *
     * @param speeds Supplier that provides the desired chassis speeds each loop.
     * @return A RunCommand that drives the swerve robot-relative.
     */
    public Command getRobotRelativeVelocityCommand(Supplier<ChassisSpeeds> speeds) {
        return run(() -> driveWithVelocityRobotRelative(speeds.get()));
    }

    /**
     * Returns a command that drives the swerve at a constant field-relative velocity.
     *
     * @param speeds Constant chassis speeds to drive.
     * @return A RunCommand that drives the swerve field-relative.
     */
    public Command getVelocityCommand(ChassisSpeeds speeds) {
        return run(() -> driveWithVelocity(speeds));
    }

    /**
     * Returns a command that drives the swerve at a constant robot-relative velocity.
     *
     * @param speeds Constant chassis speeds to drive.
     * @return A RunCommand that drives the swerve robot-relative.
     */
    public Command getRobotRelativeVelocityCommand(ChassisSpeeds speeds) {
        return run(() -> driveWithVelocityRobotRelative(speeds));
    }

    /**
     * Returns a command that stops the swerve drive completely.
     *
     * @return A RunCommand that stops all motion of the swerve.
     */
    public Command getStopCommand() {
        return runOnce(() -> stop());
    }

    // ======================================================================
    // CONTINUOUS DRIVE-TO-POSE
    // ======================================================================

    /**
     * Continuously drives toward the given fixed target pose using the subsystem's
     * {@link HolonomicDriveController}. This command never finishes on its own;
     * it simply computes chassis speeds every cycle based on the current pose and
     * the target pose.
     *
     * <p>This static version always drives toward the same pose.
     *
     * @param targetPose The fixed pose the robot should continuously drive toward.
     * @return A command that continuously applies controller output toward the pose.
     */
    public Command getContinuousDriveToPoseCommand(Pose2d targetPose) {
        return getContinuousDriveToPoseCommand(() -> targetPose);
    }

    /**
     * Continuously drives toward the given target pose, supplied dynamically.
     * This command never finishes on its own.
     *
     * <p>The supplier is evaluated **every loop**, meaning this version tracks a
     * target that may move during execution.
     *
     * @param targetPoseSupplier Supplier providing the target pose each cycle.
     * @return A command that continuously drives toward the supplied pose.
     */
    public Command getContinuousDriveToPoseCommand(Supplier<Pose2d> targetPoseSupplier) {

        return run(() -> driveToPosition(targetPoseSupplier.get()));

    }



    // ======================================================================
    // DRIVE-TO-POSE WITH TOLERANCES
    // ======================================================================

    /**
     * Drives to a fixed pose and finishes when both translational and rotational
     * tolerances are satisfied.
     *
     * <p>This is the static version; the target pose does not change during execution.
     *
     * @param targetPose The pose to drive to.
     * @param translationalTolerance Maximum allowed linear error in meters.
     * @param rotationalTolerance Maximum allowed angular error.
     * @return A terminating drive-to-pose command.
     */
    public Command getDriveToPoseCommand(
        Pose2d targetPose,
        double translationalTolerance,
        Rotation2d rotationalTolerance
    ) {
        return getDriveToPoseCommand(() -> targetPose, translationalTolerance, rotationalTolerance);
    }

    /**
     * Drives to a dynamically supplied pose and finishes when both translational
     * and rotational tolerances are satisfied.
     *
     * <p>The target pose supplier is evaluated each cycle, allowing a moving target.
     *
     * @param targetPoseSupplier Supplier providing the current target pose.
     * @param translationalTolerance Maximum allowed linear error in meters.
     * @param rotationalTolerance Maximum allowed angular error.
     * @return A terminating drive-to-pose command.
     */
    public Command getDriveToPoseCommand(
        Supplier<Pose2d> targetPoseSupplier,
        double translationalTolerance,
        Rotation2d rotationalTolerance
    ) {

        return getContinuousDriveToPoseCommand(targetPoseSupplier).until(() -> {

            Pose2d target = targetPoseSupplier.get();
            Pose2d current = getPose();

            // Check linear distance error.
            boolean withinTranslation =
                current.getTranslation()
                    .getDistance(target.getTranslation())
                    < translationalTolerance;

            // Check rotational error (absolute angular difference).
            boolean withinRotation =
                Math.abs(
                    current.getRotation()
                        .minus(target.getRotation())
                        .getRadians()
                ) < rotationalTolerance.getRadians();

            return withinTranslation && withinRotation;
        }).andThen(getStopCommand());
    }

    /**
     * Static convenience version using default tolerances.
     *
     * @param targetPose The fixed target pose.
     * @return A terminating drive-to-pose command.
     */
    public Command getDriveToPoseCommand(Pose2d targetPose) {
        return getDriveToPoseCommand(
            targetPose,
            this.translationalTolerance,
            this.rotationalTolerance
        );
    }

    /**
     * Dynamic convenience version using default tolerances.
     *
     * @param targetPoseSupplier Supplier for the pose.
     * @return A terminating drive-to-pose command.
     */
    public Command getDriveToPoseCommand(Supplier<Pose2d> targetPoseSupplier) {
        return getDriveToPoseCommand(
            targetPoseSupplier,
            this.translationalTolerance,
            this.rotationalTolerance
        );
    }



    // ======================================================================
    // PATHFIND-TO-POSE
    // ======================================================================

    /**
     * Pathfinds to a fixed target pose using the PathPlanner AutoBuilder,
     * then switches to a holonomic drive-to-pose to precisely finish alignment.
     *
     * <p>This static version always pathfinds to the same pose.
     *
     * @param targetPose The fixed pose to pathfind to.
     * @param translationalTolerance Allowed linear error for finishing.
     * @param rotationalTolerance Allowed angular error for finishing.
     * @return A command that pathfinds and then fine-aligns using holonomic control.
     */
    public Command getPathfindToPoseCommand(
        Pose2d targetPose,
        double translationalTolerance,
        Rotation2d rotationalTolerance
    ) {

        return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(
                Constants.Drive.MAX_TRANSLATIONAL_SPEED,
                Constants.Drive.MAX_TRANSLATIONAL_ACCELERATION,
                Constants.Drive.MAX_ROTATIONAL_SPEED,
                Constants.Drive.MAX_ROTATIONAL_ACCELERATION
            )
        ).andThen(
            // Precise alignment phase
            getDriveToPoseCommand(targetPose, translationalTolerance, rotationalTolerance)
        );
    }

    /**
     * Dynamic version — the Supplier is evaluated **once at command initialization**.
     *
     * <p>The snapshot is then passed to the static version to ensure pathfinding
     * stays consistent the entire run.
     *
     * @param targetPoseSupplier A supplier for the desired pose.
     * @param translationalTolerance Allowed linear error.
     * @param rotationalTolerance Allowed angular error.
     * @return A command that snapshots the target pose at init and pathfinds there.
     */
    public Command getPathfindToPoseCommand(
        Supplier<Pose2d> targetPoseSupplier,
        double translationalTolerance,
        Rotation2d rotationalTolerance
    ) {

        return new DeferredCommand(() -> {
            // Snapshot pose ONCE at init.
            Pose2d snapshot = targetPoseSupplier.get();
            return getPathfindToPoseCommand(snapshot, translationalTolerance, rotationalTolerance);
        }, Set.of(this));
    }

    /**
     * Static convenience version using default tolerances.
     *
     * @param targetPose The fixed target pose.
     * @return A command that pathfinds to the pose and then drive-to-pose finishes.
     */
    public Command getPathfindToPoseCommand(Pose2d targetPose) {
        return getPathfindToPoseCommand(
            targetPose,
            this.translationalTolerance,
            this.rotationalTolerance
        );
    }

    /**
     * Dynamic convenience version using default tolerances.
     *
     * @param targetPoseSupplier Supplier for a pose.
     * @return A command that snapshots the pose then pathfinds.
     */
    public Command getPathfindToPoseCommand(Supplier<Pose2d> targetPoseSupplier) {
        return getPathfindToPoseCommand(
            targetPoseSupplier,
            this.translationalTolerance,
            this.rotationalTolerance
        );
    }

    // ----------------------------------------------------------------------
    // DRIVE TRANSLATION WITH TARGET ROTATION
    // ----------------------------------------------------------------------

    /**
     * Drives the robot using supplied chassis velocities (translation) while
     * automatically turning toward a target rotation.
     * Static version.
     *
     * @param speeds Chassis velocities (vx, vy).
     * @param targetRotation Target rotation to face.
     * @return A command driving translation while controlling rotation automatically.
     */
    public Command driveTranslationWithTargetRotation(Translation2d speeds, Rotation2d targetRotation) {
        return driveTranslationWithTargetRotation(() -> speeds, () -> targetRotation);
    }

    /**
     * Drives the robot using dynamic chassis velocities (translation) while
     * automatically turning toward a dynamically supplied target rotation.
     * Dynamic version.
     *
     * @param speedsSupplier Supplier of Chassis velocities (vx, vy).
     * @param targetRotationSupplier Supplier of target rotation to face.
     * @return A command driving translation while controlling rotation automatically.
     */
    public Command driveTranslationWithTargetRotation(
            Supplier<Translation2d> speedsSupplier,
            Supplier<Rotation2d> targetRotationSupplier) {

        return run(() -> driveWithCompositeRequests(
            new TranslationRequest.Velocity(speedsSupplier.get(),true), 
            new RotationRequest.Position(targetRotationSupplier.get())
        ));
    }

    /**
     * Drives the robot using supplied chassis velocities (translation) while
     * automatically turning toward a target rotation.
     * Robot-relative static version.
     *
     * @param speeds Chassis velocities (vx, vy) relative to the robot.
     * @param targetRotation Target rotation to face.
     * @return A command driving translation while controlling rotation automatically.
     */
    public Command driveTranslationWithTargetRotationRobotRelative(Translation2d speeds, Rotation2d targetRotation) {
        return driveTranslationWithTargetRotationRobotRelative(() -> speeds, () -> targetRotation);
    }

    /**
     * Drives the robot using dynamic chassis velocities (translation) while
     * automatically turning toward a dynamically supplied target rotation.
     * Robot-relative dynamic version.
     *
     * @param speedsSupplier Supplier of chassis velocities (vx, vy) relative to the robot.
     * @param targetRotationSupplier Supplier of target rotation to face.
     * @return A command driving translation while controlling rotation automatically.
     */
    public Command driveTranslationWithTargetRotationRobotRelative(
            Supplier<Translation2d> speedsSupplier,
            Supplier<Rotation2d> targetRotationSupplier) {

        return run(() -> driveWithCompositeRequests(
            new TranslationRequest.Velocity(speedsSupplier.get(), false), // robot-relative
            new RotationRequest.Position(targetRotationSupplier.get())
        ));
    }


    // ----------------------------------------------------------------------
    // DRIVE ROTATION WITH TARGET TRANSLATION
    // ----------------------------------------------------------------------

    /**
     * Drives the robot using supplied chassis velocities (rotation) while
     * automatically moving toward a target translation.
     * Static version.
     *
     * @param speeds Chassis velocities (rotation only).
     * @param targetTranslation Translation target to move toward.
     * @return A command driving rotation while controlling translation automatically.
     */
    public Command driveRotationWithTargetTranslation(double speeds, Translation2d targetTranslation) {
        return driveRotationWithTargetTranslation(() -> speeds, () -> targetTranslation);
    }

    /**
     * Drives the robot using dynamic chassis velocities (rotation) while
     * automatically moving toward a dynamically supplied translation target.
     * Dynamic version.
     *
     * @param speedsSupplier Supplier of chassis velocities (rotation only).
     * @param targetTranslationSupplier Supplier of translation target.
     * @return A command driving rotation while controlling translation automatically.
     */
    public Command driveRotationWithTargetTranslation(
            Supplier<Double> speedsSupplier,
            Supplier<Translation2d> targetTranslationSupplier) {

        return run(() -> driveWithCompositeRequests(
            new TranslationRequest.Position(targetTranslationSupplier.get()), 
            new RotationRequest.Velocity(speedsSupplier.get())
        ));
    }

}

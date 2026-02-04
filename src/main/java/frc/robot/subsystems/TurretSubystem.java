package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.turret.TurretSolver;
import frc.robot.util.turret.TurretUtil;

/**
 * Subsystem responsible for controlling the robot's turret yaw.
 *
 * <p>This subsystem:
 * <ul>
 *   <li>Controls a closed-loop motor for turret rotation</li>
 *   <li>Tracks whether the turret is within an angular tolerance of its target</li>
 *   <li>Supports both robot-relative and field-relative aiming</li>
 *   <li>Integrates with {@link TurretSolver} for dynamic target tracking</li>
 * </ul>
 *
 * <p>All angles are represented using {@link Rotation2d}. Internally, the turret
 * is controlled in robot-relative coordinates.
 */
public class TurretSubystem extends SubsystemBase {

    // --------------------------------------------------------------------
    // Hardware
    // --------------------------------------------------------------------

    /** Brushless motor driving the turret rotation. */
    private final SparkMax turretMotor;

    /** Configuration object for the turret motor controller. */
    private final SparkMaxConfig turretConfig;

    // --------------------------------------------------------------------
    // Robot State Suppliers
    // --------------------------------------------------------------------

    /**
     * Supplies the robot's current estimated pose.
     * Defaults to zero pose until overridden.
     */
    private Supplier<Pose2d> robotPoseSupplier = () -> Pose2d.kZero;

    /**
     * Supplies the robot's current field-relative chassis speeds.
     * Defaults to zero velocity until overridden.
     */
    private Supplier<ChassisSpeeds> robotVelocitySupplier = ChassisSpeeds::new;

    // --------------------------------------------------------------------
    // Internal State
    // --------------------------------------------------------------------

    /**
     * True if the turret is currently within tolerance of its commanded angle.
     * This value is only valid while an angle command is running.
     */
    private boolean atTarget = false;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    /**
     * Creates the turret subsystem and configures the motor controller.
     */
    public TurretSubystem() {

        turretConfig = new SparkMaxConfig();

        turretConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Turret.TURRET_MOTOR_CURRENT_LIMIT)
            .inverted(false);

        turretConfig.encoder
            .positionConversionFactor(Constants.Turret.ENCODER_FACTOR)
            .velocityConversionFactor(Constants.Turret.ENCODER_FACTOR / 60.0);

        turretConfig.closedLoop.pid(
            Constants.Turret.TURRET_KP,
            Constants.Turret.TURRET_KI,
            Constants.Turret.TURRET_KD
        );

        turretMotor = new SparkMax(
            Constants.Turret.TURRET_ID,
            MotorType.kBrushless
        );

        turretMotor.configure(
            turretConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );
    }

    // --------------------------------------------------------------------
    // Turret Angle Control
    // --------------------------------------------------------------------

    /**
     * Commands the turret to rotate to a robot-relative angle.
     *
     * <p>The requested angle is clamped to the allowed mechanical range,
     * selecting the closest valid equivalent rotation.</p>
     *
     * @param angle desired turret yaw (robot-relative)
     */
    public void setTurretAngle(Rotation2d angle) {

        Rotation2d clampedAngle =
            TurretUtil.resolveClosestValidAngle(
                getAngle(),
                angle,
                Constants.Turret.MIN_ROTATION,
                Constants.Turret.MAX_ROTATION
            );

        turretMotor
            .getClosedLoopController()
            .setSetpoint(
                clampedAngle.getRadians(),
                com.revrobotics.spark.SparkBase.ControlType.kPosition
            );

    }

    /**
     * Commands the turret to rotate to a field-relative yaw.
     *
     * <p>This automatically compensates for the robot's current heading.</p>
     *
     * @param fieldAngle desired turret yaw in the field frame
     */
    public void setTurretAngleFieldRelative(Rotation2d fieldAngle) {
        Rotation2d robotHeading = robotPoseSupplier.get().getRotation();
        setTurretAngle(fieldAngle.minus(robotHeading));
    }

    /**
     * @return the current turret yaw (robot-relative)
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(
            turretMotor.getEncoder().getPosition()
        );
    }

    /**
     * @return the current turret yaw in the field frame
     */
    public Rotation2d getAngleFieldRelative() {
        return getAngle().plus(robotPoseSupplier.get().getRotation());
    }

    // --------------------------------------------------------------------
    // Robot State Injection
    // --------------------------------------------------------------------

    /**
     * Sets the supplier used to obtain the robot's estimated pose.
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    /**
     * Sets the supplier used to obtain the robot's chassis speeds.
     */
    public void setRobotVelocitySupplier(
        Supplier<ChassisSpeeds> robotVelocitySupplier
    ) {
        this.robotVelocitySupplier = robotVelocitySupplier;
    }

    // --------------------------------------------------------------------
    // Commands
    // --------------------------------------------------------------------

    /**
     * Creates a command that continuously drives the turret toward a
     * robot-relative target angle.
     *
     * <p>While running, the command updates an internal {@code atTarget} flag
     * whenever the angular error is within the configured tolerance.
     * The flag is cleared when the command ends.</p>
     *
     * @param angleSupplier supplies the desired turret angle (robot-relative)
     * @return a continuously running turret-aiming command
     */
    public Command getAngleCommand(
        Supplier<Rotation2d> angleSupplier
    ) {
        return new FunctionalCommand(
            () -> {},

            () -> {
                Rotation2d targetAngle = angleSupplier.get();
                setTurretAngle(targetAngle);

                atTarget =
                    Math.abs(
                        getAngle()
                            .minus(targetAngle)
                            .getRadians()
                    ) < Constants.Turret.TURRET_TOLERANCE.getRadians();
            },

            interrupted -> atTarget = false,
            () -> false,
            this
        );
    }

    /**
     * Creates a command that automatically aims the turret at a field-relative
     * 3D target using the ballistic {@link TurretSolver}.
     *
     * <p>The solver accounts for robot motion and returns the required yaw.</p>
     *
     * @param targetTranslationSupplier supplies the field-relative target position
     * @return a turret-aiming command driven by the solver
     */
    public Command getTargetCommand(
        Supplier<Translation3d> targetTranslationSupplier
    ) {
        return getAngleCommand(() -> {
            TurretSolver.State solution =
                TurretSolver.solve(
                    robotPoseSupplier.get(),
                    robotVelocitySupplier.get(),
                    targetTranslationSupplier.get(),
                    Constants.Turret.SOLVER_CONFIG
                );

            return solution.getLaunchAngle();
        });
    }

    // --------------------------------------------------------------------
    // Status
    // --------------------------------------------------------------------

    /**
     * @return true if the turret is currently within tolerance of its target
     */
    public boolean atTarget() {
        return atTarget;
    }
}

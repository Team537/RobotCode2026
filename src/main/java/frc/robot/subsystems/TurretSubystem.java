package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
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
    private final TalonFX turretMotor;
    private final PWM pitchServo = new PWM(Constants.Turret.PITCH_SERVO_ID);
    private final CANcoder pitchEncoder = new CANcoder(Constants.Turret.PITCH_CANCODER_ID);

    private volatile double hoodSetpointRad = Constants.Turret.MIN_PITCH.getRadians();
    private volatile boolean hoodClosedLoopActive = false;

    PIDController hoodController = new PIDController(Constants.Turret.PITCH_KP, Constants.Turret.PITCH_KI, Constants.Turret.PITCH_KD);

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
        turretMotor = new TalonFX(Constants.Turret.TURRET_ID);
        turretMotor.getConfigurator().apply(Configs.TURRET_CONFIG);
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

        PositionVoltage positionRequest = new PositionVoltage(clampedAngle.getRadians());
        turretMotor.setControl(positionRequest);

    }

    public void setHoodAngle(Rotation2d angle) {
        double minR = Constants.Turret.ENCODER_MIN_PITCH.getRadians();
        double maxR = Constants.Turret.ENCODER_MAX_PITCH.getRadians();

        double clamped = Math.max(minR, Math.min(maxR, angle.getRadians()));

        hoodSetpointRad = clamped;
        hoodController.reset();
        hoodController.setSetpoint(hoodSetpointRad);
        hoodClosedLoopActive = true;
    }

    /**
     * Commands the turret to rotate to a field-relative yaw.
     *
     * <p>This automatically compensates for the robot's current heading.</p>
     *
     * @param fieldAngle desired turret yaw in the field frame
     */
    public void setTurretAngleFieldRelative(Rotation2d fieldAngle, Rotation2d robotRotation) {
        Rotation2d robotHeading = robotRotation;
        setTurretAngle(fieldAngle.minus(robotHeading));
    }

    /**
     * @return the current turret yaw (robot-relative)
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turretMotor.getPosition().getValueAsDouble());
    }


    public Rotation2d getHoodAngle() {
        return Rotation2d.fromRadians(pitchEncoder.getPosition().getValueAsDouble() * Constants.Turret.PITCH_ENCODER_FACTOR);
    }

    /**
     * @return the current turret yaw in the field frame
     */
    public Rotation2d getAngleFieldRelative(Rotation2d robotRotation) {
        return getAngle().plus(robotRotation);
    }

    /**
     * Sets the position of the turret
     * @param rotation The position to set to
     */
    public void resetTurretAngle(Rotation2d rotation) {
        turretMotor.setPosition(rotation.getRadians());
    }


    public void periodic() {
        if (!hoodClosedLoopActive) {
            return;
        }

        double current = getHoodAngle().getRadians();
        double output = hoodController.calculate(current);

        double limited = Math.max(-0.4, Math.min(0.4, output));
        pitchServo.setSpeed(limited);

        if (hoodController.atSetpoint()) {
            hoodClosedLoopActive = false;
        }
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
        Supplier<Translation3d> targetTranslationSupplier,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier
    ) {
        return getAngleCommand(() -> {
            TurretSolver.State solution =
                TurretSolver.solve(
                    robotPoseSupplier.get(),
                    robotVelocitySupplier.get(),
                    targetTranslationSupplier.get(),
                    Constants.Turret.SOLVER_CONFIG
                );

            return solution.getYaw();
        });
    }

    public Command setHoodAngleCommand(Supplier<Rotation2d> angleSupplier) {
        return new FunctionalCommand(
            () -> {},

            () -> setHoodAngle(angleSupplier.get()),

            interrupted -> {},
            () -> false
        );
    }

    public Command resetHoodAngleCommand(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {},

            () -> setHoodAngle(angle),
            interrupted -> {},
            () -> false
        );
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

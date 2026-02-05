package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.util.turret.TurretSolver;
import frc.robot.util.turret.TurretUtil;

/**
 * Subsystem responsible for controlling the shooter flywheel(s).
 *
 * <p>This subsystem:
 * <ul>
 *   <li>Controls shooter wheel velocity using closed-loop control</li>
 *   <li>Tracks whether the shooter is within velocity tolerance of its target</li>
 *   <li>Provides commands that continuously maintain a desired speed</li>
 * </ul>
 *
 * <p>The {@code atTarget} state is only valid while a velocity command
 * is actively running.</p>
 */
public class ShooterSubsystem extends SubsystemBase {

    // --------------------------------------------------------------------
    // Hardware
    // --------------------------------------------------------------------

    /** TalonFX driving the shooter wheel(s). */
    private final TalonFX shooterMotor;

    // --------------------------------------------------------------------
    // Internal State
    // --------------------------------------------------------------------

    /**
     * True if the shooter velocity is currently within tolerance of
     * its commanded setpoint.
     */
    private boolean atTarget = false;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    /**
     * Creates the shooter subsystem and applies motor configuration.
     */
    public ShooterSubsystem() {
        shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_ID);
        shooterMotor
            .getConfigurator()
            .apply(Configs.Shooter.SHOOTER_CONFIGURATION);
    }

    // --------------------------------------------------------------------
    // Shooter Control
    // --------------------------------------------------------------------

    /**
     * Commands the shooter flywheel to a target velocity.
     *
     * @param velocity desired wheel velocity in meters per second
     */
    public void setRawVelocity(double velocity) {
        shooterMotor.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Commands the shooter flywheel so the ball with shoot out at the specified velocity
     *
     * @param velocity desired ball velocity in meters per second
     */
    public void setVelocity(double velocity) {
        setRawVelocity(TurretUtil.wheelSurfaceSpeedFromBallSpeed(velocity));
    }

    /**
     * @return the current shooter flywheel velocity in meters per second
     */
    public double getRawVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @return the current velocity that a ball shot out would travel in meters per second
     */
    public double getVelocity() {
        return TurretUtil.ballSpeedFromWheelSurfaceSpeed(getRawVelocity());
    }

    // --------------------------------------------------------------------
    // Commands
    // --------------------------------------------------------------------

    /**
     * Creates a command that continuously drives the shooter's raw flywheel speed
     * toward a target velocity.
     *
     * <p>While running, the command updates an internal {@code atTarget}
     * flag whenever the velocity error is within the configured tolerance.
     * The flag is cleared when the command ends.</p>
     *
     * @param velocitySupplier supplies the desired raw shooter velocity
     * @return a continuously running shooter velocity command
     */
    public Command getRawVelocityCommand(
        Supplier<Double> velocitySupplier
    ) {
        return new FunctionalCommand(
            () -> {},

            () -> {
                double targetVelocity = velocitySupplier.get();
                setRawVelocity(targetVelocity);

                atTarget =
                    Math.abs(
                        targetVelocity - getRawVelocity()
                    ) < Constants.Shooter.TOLERANCE;
            },

            interrupted -> atTarget = false,
            () -> false,
            this
        );
    }

    /**
     * Creates a command that continuously drives the shooter toward a
     * target velocity.
     *
     * <p>While running, the command updates an internal {@code atTarget}
     * flag whenever the velocity error is within the configured tolerance.
     * The flag is cleared when the command ends.</p>
     *
     * @param velocitySupplier supplies the desired shooter velocity
     * @return a continuously running shooter velocity command
     */
    public Command getVelocityCommand(
        Supplier<Double> velocitySupplier
    ) {
        return new FunctionalCommand(
            () -> {},

            () -> {
                double targetVelocity = velocitySupplier.get();
                setVelocity(targetVelocity);

                atTarget =
                    Math.abs(
                        targetVelocity - getVelocity()
                    ) < Constants.Shooter.TOLERANCE;
            },

            interrupted -> atTarget = false,
            () -> false,
            this
        );
    }

    /**
     * Creates a command that stops the shooter and ends when the shooter velocity is zero.
     * @return A command to stop the shooter.
     */
    public Command getStopCommand() {
        return getVelocityCommand(() -> 0.0).until(() -> atTarget);
    }

    /**
     * Creates a command that automatically sets the shooter wheel velocity
     * required to hit a field-relative 3D target.
     *
     * <p>This command uses the ballistic {@link TurretSolver} to compute the
     * necessary launch velocity based on:
     * <ul>
     *   <li>The robot's current pose</li>
     *   <li>The robot's current chassis velocity</li>
     *   <li>The field-relative position of the target</li>
     * </ul>
     *
     * <p>The solver accounts for robot motion and returns the required
     * shooter launch velocity, which is continuously commanded while the
     * command is running.</p>
     *
     * @param targetTranslationSupplier supplies the field-relative target position
     * @param robotPoseSupplier supplies the robot's current field pose
     * @param robotVelocitySupplier supplies the robot's current chassis speeds
     * @return a shooter velocity command driven by the ballistic solver
     */
    public Command getTargetCommand(
        Supplier<Translation3d> targetTranslationSupplier,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier
    ) {
        return getVelocityCommand(() -> {
            TurretSolver.State solution =
                TurretSolver.solve(
                    robotPoseSupplier.get(),
                    robotVelocitySupplier.get(),
                    targetTranslationSupplier.get(),
                    Constants.Turret.SOLVER_CONFIG
                );

            return solution.getLaunchVelocity();
        });
    }


    // --------------------------------------------------------------------
    // Status
    // --------------------------------------------------------------------

    /**
     * @return true if the shooter is currently within velocity tolerance
     *         of its target
     */
    public boolean atTarget() {
        return atTarget;
    }
}

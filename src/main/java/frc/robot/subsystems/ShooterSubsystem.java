package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final TalonFX leadShooterMotor;
    //private final TalonFX followerShooterMotor;

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
        leadShooterMotor = new TalonFX(Constants.Shooter.LEAD_SHOOTER_ID);
        leadShooterMotor
            .getConfigurator()
            .apply(Configs.Shooter.SHOOTER_CONFIGURATION);
        leadShooterMotor.setPosition(0.0);

        /*followerShooterMotor = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_ID);
        followerShooterMotor
            .getConfigurator()
            .apply(Configs.Shooter.SHOOTER_CONFIGURATION);
        followerShooterMotor.setPosition(0.0);*/
    }

    // --------------------------------------------------------------------
    // Shooter Control
    // --------------------------------------------------------------------

    /**
     * Commands the shooter flywheel to a target velocity.
     *
     * @param velocity desired wheel velocity in meters per second
     */

    public void setWheelVelocity(double velocity) {
        leadShooterMotor.setControl(new VelocityVoltage(velocity));
        //followerShooterMotor.setControl(new Follower(Constants.Shooter.LEAD_SHOOTER_ID,MotorAlignmentValue.Opposed));
        SmartDashboard.putNumber("Target Shooter Velocity (Wheel)", velocity);
    }
    /**
     * Commands the shooter flywheel so the ball with shoot out at the specified velocity
     *
     * @param velocity desired ball velocity in meters per second
     */
    public void setBallVelocity(double velocity) {
        setWheelVelocity(TurretUtil.wheelSurfaceSpeedFromBallSpeed(velocity));
    }

    /**
     * @return the current shooter flywheel velocity in meters per second
     */
    public double getWheelVelocity() {
        return leadShooterMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @return the current velocity that a ball shot out would travel in meters per second
     */
    public double getBallVelocity() {
        return TurretUtil.ballSpeedFromWheelSurfaceSpeed(getWheelVelocity());
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
    public Command getWheelVelocityCommand(
        Supplier<Double> velocitySupplier
    ) {
        return new FunctionalCommand(
            () -> {},

            () -> {
                double targetVelocity = velocitySupplier.get();
                setWheelVelocity(targetVelocity);

                atTarget =
                    Math.abs(
                        targetVelocity - getWheelVelocity()
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
    public Command getBallVelocityCommand(
        Supplier<Double> velocitySupplier
    ) {
        return new FunctionalCommand(
            () -> {},

            () -> {
                double targetVelocity = velocitySupplier.get();
                setBallVelocity(targetVelocity);

                atTarget =
                    Math.abs(
                        targetVelocity - getBallVelocity()
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
        return new RunCommand(() -> {
            leadShooterMotor.stopMotor();
            //followerShooterMotor.stopMotor();
        },this);
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
        return getBallVelocityCommand(() -> {
            TurretSolver.State solution =
                TurretSolver.solve(
                    robotPoseSupplier.get(),
                    robotVelocitySupplier.get(),
                    targetTranslationSupplier.get(),
                    Constants.Turret.SOLVER_CONFIG
                );

            SmartDashboard.putNumber("Target Max Height", solution.getMaxHeight());
            SmartDashboard.putNumber("Target Impact Velocity", solution.getImpactVelocity());
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Wheel Velocity",getWheelVelocity());
        //SmartDashboard.putNumber("Current Wheel Velocity Follower",followerShooterMotor.getVelocity().getValueAsDouble());
    }

}

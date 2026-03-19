package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.util.turret.TurretSolver;
import frc.robot.util.turret.TurretUtil;

/**
 * Subsystem responsible for controlling the robot's turret yaw.
 *
 * <p>
 * This subsystem:
 * <ul>
 * <li>Controls a closed-loop motor for turret rotation</li>
 * <li>Tracks whether the turret is within an angular tolerance of its
 * target</li>
 * <li>Supports both robot-relative and field-relative aiming</li>
 * <li>Integrates with {@link TurretSolver} for dynamic target tracking</li>
 * </ul>
 *
 * <p>
 * All angles are represented using {@link Rotation2d}. Internally, the turret
 * is controlled in robot-relative coordinates.
 */
public class TurretSubsystem extends SubsystemBase {

    // --------------------------------------------------------------------
    // Hardware
    // --------------------------------------------------------------------

    /** Brushless motor driving the turret rotation. */
    private final TalonFX turretMotor;
    private final PWM pitchServo;
    private final CANcoder pitchEncoder;

    private volatile double hoodSetpointRad = Constants.Turret.MIN_PITCH.getRadians();
    private volatile boolean hoodClosedLoopActive = false;

    private Supplier<Rotation2d> turretOffsetSupplier = () -> Rotation2d.kZero;
    private Supplier<Rotation2d> hoodOffsetSupplier = () -> Rotation2d.kZero;

    PIDController hoodController = new PIDController(Constants.Turret.PITCH_KP, Constants.Turret.PITCH_KI,
            Constants.Turret.PITCH_KD);

    // Shadow values used to detect dashboard-driven PID changes.
    private double lastHoodKp = Constants.Turret.PITCH_KP;
    private double lastHoodKi = Constants.Turret.PITCH_KI;
    private double lastHoodKd = Constants.Turret.PITCH_KD;

    // SmartDashboard keys for the hood PID gains.
    private static final String HOOD_KP_KEY = "Hood PID/kP";
    private static final String HOOD_KI_KEY = "Hood PID/kI";
    private static final String HOOD_KD_KEY = "Hood PID/kD";

    // --------------------------------------------------------------------
    // Internal State
    // --------------------------------------------------------------------

    /**
     * True if the turret is currently within tolerance of its commanded angle.
     * This value is only valid while an angle command is running.
     * same applies to the hood
     */
    private boolean atTarget = false;
    private boolean atHoodTarget = false;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    /**
     * Creates the turret subsystem and configures the motor controller.
     */
    public TurretSubsystem() {
        turretMotor = new TalonFX(Constants.Turret.TURRET_ID, Constants.CANIVORE_LOOP_NAME);
        turretMotor.getConfigurator().apply(Configs.TURRET_CONFIG);
        resetTurretAngle(Constants.Turret.START_POSITION);
        pitchServo = new PWM(Constants.Turret.PITCH_SERVO_ID);
        pitchEncoder = new CANcoder(Constants.Turret.PITCH_CANCODER_ID, Constants.CANIVORE_LOOP_NAME);
        resetHoodAngle(Constants.Turret.HOOD_START_POSITION);

        // Publish default hood PID gains so they appear as editable fields
        // in Elastic / AdvantageScope / Shuffleboard without overwriting
        // any existing persisted/tuned values.
        SmartDashboard.setDefaultNumber(HOOD_KP_KEY, Constants.Turret.PITCH_KP);
        SmartDashboard.setDefaultNumber(HOOD_KI_KEY, Constants.Turret.PITCH_KI);
        SmartDashboard.setDefaultNumber(HOOD_KD_KEY, Constants.Turret.PITCH_KD);
    }

    // --------------------------------------------------------------------
    // Turret Angle Control
    // --------------------------------------------------------------------

    /**
     * Commands the turret to rotate to a robot-relative angle.
     *
     * <p>
     * The requested angle is clamped to the allowed mechanical range,
     * selecting the closest valid equivalent rotation.
     * </p>
     *
     * @param angle desired turret yaw (robot-relative)
     */
    public void setTurretAngle(Rotation2d angle) {

        Rotation2d clampedAngle = TurretUtil.resolveClosestValidAngle(
                getAngle(),
                angle,
                Constants.Turret.MIN_ROTATION,
                Constants.Turret.MAX_ROTATION);

        PositionVoltage positionRequest = new PositionVoltage(
                clampedAngle.getRadians() + turretOffsetSupplier.get().getRadians());
        turretMotor.setControl(positionRequest);
        SmartDashboard.putNumber("Turret Target", clampedAngle.getDegrees());

    }

    public void setHoodAngle(Rotation2d angle) {
        double minR = Constants.Turret.MIN_PITCH.getRadians();
        double maxR = Constants.Turret.MAX_PITCH.getRadians();

        double clamped = Math.max(minR, Math.min(maxR, angle.getRadians()));

        hoodSetpointRad = clamped;
        hoodController.setSetpoint(hoodSetpointRad + hoodOffsetSupplier.get().getRadians());
        hoodClosedLoopActive = true;

        SmartDashboard.putNumber("Hood Target", Rotation2d.fromRadians(clamped).getDegrees());
    }

    /**
     * Commands the turret to rotate to a field-relative yaw.
     *
     * <p>
     * This automatically compensates for the robot's current heading.
     * </p>
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
        return Rotation2d
                .fromRadians(turretMotor.getPosition().getValueAsDouble() - turretOffsetSupplier.get().getRadians());
    }

    public Rotation2d getHoodAngle() {
        return Rotation2d
                .fromRadians(pitchEncoder.getPosition().getValueAsDouble() * Constants.Turret.PITCH_ENCODER_FACTOR
                        - hoodOffsetSupplier.get().getRadians());
    }

    /**
     * @return the current turret yaw in the field frame
     */
    public Rotation2d getAngleFieldRelative(Rotation2d robotRotation) {
        return getAngle().plus(robotRotation);
    }

    /**
     * Sets the position of the turret
     * 
     * @param rotation The position to set to
     */
    public void resetTurretAngle(Rotation2d rotation) {
        turretMotor.setPosition(rotation.getRadians() + turretOffsetSupplier.get().getRadians());
    }

    /**
     * Sets the position of the hood
     * 
     * @param rotation The position to set to
     */
    public void resetHoodAngle(Rotation2d rotation) {
        pitchEncoder.setPosition(
                rotation.getRadians() / Constants.Turret.PITCH_ENCODER_FACTOR + hoodOffsetSupplier.get().getRadians());
    }

    public void stopTurretMotor() {
        turretMotor.stopMotor();
    }

    public void stopHoodServo() {
        hoodClosedLoopActive = false;
        pitchServo.setSpeed(0);
    }

    public void periodic() {

        SmartDashboard.putNumber("Current Turret Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Current Hood Angle", getHoodAngle().getDegrees());
        SmartDashboard.putNumber("Current Hood Speed", pitchServo.getSpeed());

        // Read PID gains from the dashboard and update the controller if anything changed.
        double dashKp = SmartDashboard.getNumber(HOOD_KP_KEY, lastHoodKp);
        double dashKi = SmartDashboard.getNumber(HOOD_KI_KEY, lastHoodKi);
        double dashKd = SmartDashboard.getNumber(HOOD_KD_KEY, lastHoodKd);

        if (dashKp != lastHoodKp || dashKi != lastHoodKi || dashKd != lastHoodKd) {
            boolean finite  = Double.isFinite(dashKp) && Double.isFinite(dashKi) && Double.isFinite(dashKd);
            boolean nonNeg  = dashKp >= 0.0 && dashKi >= 0.0 && dashKd >= 0.0;

            if (finite && nonNeg) {
                hoodController.setPID(dashKp, dashKi, dashKd);
                lastHoodKp = dashKp;
                lastHoodKi = dashKi;
                lastHoodKd = dashKd;
                // Reset PID internal state when gains change to avoid large transients.
                hoodController.reset();
            } else {
                // Gains are non-finite or negative — revert the dashboard entries to the
                // last known-good values so the bad value is visible but not applied.
                String reason = !finite ? "non-finite" : "negative";
                DriverStation.reportWarning(
                        "Hood PID: rejected " + reason + " gain(s) from dashboard "
                                + "(kP=" + dashKp + ", kI=" + dashKi + ", kD=" + dashKd
                                + "). Reverting to last valid values "
                                + "(kP=" + lastHoodKp + ", kI=" + lastHoodKi + ", kD=" + lastHoodKd + ").",
                        false);
                SmartDashboard.putNumber(HOOD_KP_KEY, lastHoodKp);
                SmartDashboard.putNumber(HOOD_KI_KEY, lastHoodKi);
                SmartDashboard.putNumber(HOOD_KD_KEY, lastHoodKd);
            }
        }

        if (hoodClosedLoopActive && DriverStation.isEnabled()) {
            double current = getHoodAngle().getRadians();
            double output = hoodController.calculate(current);

            if (Double.isFinite(output)) {
                pitchServo.setSpeed((Constants.Turret.PITCH_INVERTED ? -1.0 : 1.0) * output);
            } else {
                // Encoder or controller produced a non-finite output — stop the servo safely.
                DriverStation.reportWarning(
                        "Hood PID: non-finite output (" + output + ") from controller; stopping servo.", false);
                pitchServo.setSpeed(0);
            }
        }

    }

    // --------------------------------------------------------------------
    // Commands
    // --------------------------------------------------------------------

    /**
     * Creates a command that continuously drives the turret toward a
     * robot-relative target angle.
     *
     * <p>
     * While running, the command updates an internal {@code atTarget} flag
     * whenever the angular error is within the configured tolerance.
     * The flag is cleared when the command ends.
     * </p>
     *
     * @param angleSupplier supplies the desired turret angle (robot-relative)
     * @return a continuously running turret-aiming command
     */
    public Command getAngleCommand(
            Supplier<Rotation2d> angleSupplier,
            Supplier<Rotation2d> hoodAngleSupplier) {
        return new FunctionalCommand(
                () -> {
                },

                () -> {
                    Rotation2d targetAngle = angleSupplier.get();
                    setTurretAngle(targetAngle);

                    Rotation2d targetHoodAngle = hoodAngleSupplier.get();
                    setHoodAngle(targetHoodAngle);

                    atTarget = Math.abs(
                            getAngle()
                                    .minus(targetAngle)
                                    .getRadians()) < Constants.Turret.TURRET_TOLERANCE.getRadians();

                    atHoodTarget = Math.abs(
                            getHoodAngle()
                                    .minus(targetHoodAngle)
                                    .getRadians()) < Constants.Turret.HOOD_TOLERANCE.getRadians();
                },

                interrupted -> {
                    atTarget = false;
                    stopHoodServo();
                },
                () -> false,
                this);
    }

    public Command getStowCommand() {

        Command moveToTarget = new FunctionalCommand(
                () -> SmartDashboard.putBoolean("Turret/IsStowing", true),

                () -> {
                    setHoodAngle(Constants.Turret.HOOD_STOW_POSITION);
                },

                interrupted -> {
                },

                () -> Math.abs(
                        getHoodAngle()
                                .minus(Constants.Turret.HOOD_STOW_POSITION)
                                .getRadians()) < Constants.Turret.HOOD_TOLERANCE.getRadians(),

                this);

        Command settleDown = new RunCommand(
                () -> pitchServo.setSpeed((Constants.Turret.PITCH_INVERTED ? -1.0 : 1.0) * -Constants.Turret.STOW_PUSH_DOWN_SPEED), // small constant downward speed
                this).withTimeout(Constants.Turret.STOW_PUSH_DOWN_TIME); // enough to seat the gear

        Command finish = new InstantCommand(() -> {
            pitchServo.setSpeed(0.0);
        });

        return Commands.sequence(moveToTarget, settleDown, finish, Commands.idle())
                .withName("StowHood");
    }

    /**
     * Creates a command that automatically aims the turret at a field-relative
     * 3D target using the ballistic {@link TurretSolver}.
     *
     * <p>
     * The solver accounts for robot motion and returns the required yaw.
     * </p>
     *
     * @param targetTranslationSupplier supplies the field-relative target position
     * @return a turret-aiming command driven by the solver
     */
    public Command getTargetCommand(
            Supplier<Translation3d> targetTranslationSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        return getAngleCommand(
                () -> {
                    TurretSolver.State solution = TurretSolver.solve(
                            robotPoseSupplier.get(),
                            robotVelocitySupplier.get(),
                            targetTranslationSupplier.get(),
                            Constants.Turret.SOLVER_CONFIG);
                    return solution.getYaw();
                },
                () -> {
                    TurretSolver.State solution = TurretSolver.solve(
                            robotPoseSupplier.get(),
                            robotVelocitySupplier.get(),
                            targetTranslationSupplier.get(),
                            Constants.Turret.SOLVER_CONFIG);
                    return Rotation2d.fromRadians(0.5 * Math.PI).minus(solution.getPitch());
                }).withName("TargetTurret");
    }

    /**
     * Creates a command to float the motor temporarily
     * @return
     */
    public Command getFloatCommand() {
        return new FunctionalCommand(
            () -> turretMotor.setNeutralMode(NeutralModeValue.Coast),
            () -> {},
            (interrupted) -> turretMotor.setNeutralMode(NeutralModeValue.Brake), 
            () -> false,
            this
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

    public boolean atHoodTarget() {
        return atHoodTarget;
    }

    /**
     * sets the supplier of the turret offset
     * 
     * @param turretOffsetSupplier the supplier of the turret offset. 0 means no
     *                             offset
     */
    public void setTurretOffsetSupplier(Supplier<Rotation2d> turretOffsetSupplier) {
        this.turretOffsetSupplier = turretOffsetSupplier;
    }

    /**
     * sets the supplier of the hood offset
     * 
     * @param hoodOffsetSupplier the supplier of the hood offset. 0 means no
     *                           offset
     */
    public void setHoodOffsetSupplier(Supplier<Rotation2d> hoodOffsetSupplier) {
        this.hoodOffsetSupplier = hoodOffsetSupplier;
    }

}

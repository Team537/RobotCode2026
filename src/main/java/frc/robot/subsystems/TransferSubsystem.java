package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem responsible for controlling the transfer mechanism.
 *
 * <p>The transfer moves game pieces between internal subsystems
 * (e.g. intake → shooter). It is velocity-controlled using a
 * closed-loop controller on a Spark MAX.</p>
 */
public class TransferSubsystem extends SubsystemBase {

    // --------------------------------------------------------------------
    // Hardware
    // --------------------------------------------------------------------

    /** Motor driving the transfer mechanism. */
    private final SparkMax transferMotor;

    /** Configuration object applied to the transfer motor controller. */
    private final SparkMaxConfig transferConfig;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    /**
     * Creates and configures the transfer subsystem.
     */
    public TransferSubsystem() {

        transferConfig = new SparkMaxConfig();

        // Basic motor behavior
        transferConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Transfer.CURRENT_LIMIT)
            .inverted(false);

        // Encoder conversion factors
        transferConfig.encoder
            .positionConversionFactor(Constants.Transfer.ENCODER_FACTOR)
            .velocityConversionFactor(Constants.Transfer.ENCODER_FACTOR / 60.0);

        // Closed-loop velocity control gains
        transferConfig.closedLoop.pid(
            Constants.Transfer.KP,
            Constants.Transfer.KI,
            Constants.Transfer.KD
        );

        // Instantiate motor controller
        transferMotor = new SparkMax(
            Constants.Transfer.TRANSFER_MOTOR_ID,
            MotorType.kBrushless
        );

        // Apply configuration to hardware
        transferMotor.configure(
            transferConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    // --------------------------------------------------------------------
    // Control
    // --------------------------------------------------------------------

    /**
     * Commands the transfer motor to a target velocity.
     *
     * @param velocity desired velocity in mechanism units per second
     */
    public void setVelocity(double velocity) {
        transferMotor
            .getClosedLoopController()
            .setSetpoint(velocity, ControlType.kVelocity);
    }

    // --------------------------------------------------------------------
    // Commands
    // --------------------------------------------------------------------

    /**
     * Creates a command that immediately sets the transfer velocity.
     *
     * <p>This command finishes instantly and leaves the motor
     * running at the requested speed.</p>
     *
     * @param velocity desired transfer velocity
     * @return an instant command that sets motor velocity
     */
    public Command getSetVelocityCommand(double velocity) {
        return new InstantCommand(
            () -> setVelocity(velocity),
            this
        );
    }

    /**
     * @return a command that runs the transfer at the load speed
     */
    public Command getLoadCommand() {
        return getSetVelocityCommand(Constants.Transfer.LOAD_SPEED);
    }

    /**
     * @return a command that stops the transfer motor
     */
    public Command getStopCommand() {
        return getSetVelocityCommand(0.0);
    }
}

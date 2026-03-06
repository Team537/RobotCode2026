package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
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
    private final TalonFX transferMotor;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    /**
     * Creates and configures the transfer subsystem.
     */
    public TransferSubsystem() {
        transferMotor = new TalonFX(Constants.Transfer.TRANSFER_MOTOR_ID);
        transferMotor.getConfigurator().apply(Configs.TRANSFER_CONFIG);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Transfer Velocity",getVelocity());
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
        VelocityVoltage velocityRequest = new VelocityVoltage(velocity);
        transferMotor.setControl(velocityRequest);
    }

    public double getVelocity() {
        return transferMotor.getVelocity().getValueAsDouble();
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
        return getSetVelocityCommand(Constants.Transfer.LOAD_SPEED).withName("TransferLoad");
    }

    /**
     * @return a command that stops the transfer motor
     */
    public Command getStopCommand() {
        return getSetVelocityCommand(0.0).withName("TransferStop");
    }
}

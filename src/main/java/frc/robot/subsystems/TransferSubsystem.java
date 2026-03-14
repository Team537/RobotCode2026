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
        transferMotor = new TalonFX(Constants.Transfer.TRANSFER_MOTOR_ID, Constants.CANIVORE_LOOP_NAME);
        transferMotor.getConfigurator().apply(Configs.TRANSFER_CONFIG);
    }

    // --------------------------------------------------------------------
    // Control
    // --------------------------------------------------------------------

    /**
     * Commands the transfer motor to a target power.
     *
     * @param power desired power
     */
    public void setPower(double power) {
        transferMotor.set(power);
    }

    // --------------------------------------------------------------------
    // Commands
    // --------------------------------------------------------------------

    /**
     * Creates a command that immediately sets the transfer power.
     *
     * <p>This command finishes instantly and leaves the motor
     * running at the requested speed.</p>
     *
     * @param power desired transfer power
     * @return an instant command that sets motor power
     */
    public Command getSetPowerCommand(double power) {
        return new InstantCommand(
            () -> setPower(power),
            this
        );
    }

    /**
     * @return a command that runs the transfer at the load speed
     */
    public Command getLoadCommand() {
        return getSetPowerCommand(Constants.Transfer.LOAD_POWER).withName("TransferLoad");
    }

    /**
     * @return a command that stops the transfer motor
     */
    public Command getStopCommand() {
        return new InstantCommand(() -> transferMotor.stopMotor());
    }
}

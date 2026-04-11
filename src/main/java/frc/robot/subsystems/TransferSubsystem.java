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
    private final TalonFX kickerMotor;
    private final TalonFX feederMotor;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    /**
     * Creates and configures the transfer subsystem.
     */
    public TransferSubsystem() {
        kickerMotor = new TalonFX(Constants.Transfer.TRANSFER_KICKER_ID, Constants.CANIVORE_LOOP_NAME);
        kickerMotor.getConfigurator().apply(Configs.KICKER_CONFIG);

        feederMotor = new TalonFX(Constants.Transfer.TRANSFER_FEEDER_ID);
        feederMotor.getConfigurator().apply(Configs.FEEDER_CONFIG);
    }

    // --------------------------------------------------------------------
    // Control
    // --------------------------------------------------------------------

    /**
     * Commands the transfer motor to a target power.
     *
     * @param power desired power
     */
    public void setKickerPower(double power) {
        kickerMotor.set(power);
    }

    public void setFeederPower(double power) {
        feederMotor.set(power);
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
    public Command getPowerCommand(double kickerPower, double feederPower) {
        return new InstantCommand(
            () -> {
                setKickerPower(kickerPower);
                setFeederPower(feederPower);
            }
        );
    }

    public Command getLoadCommand() {
        return new InstantCommand(
            () -> {
                setKickerPower(Constants.Transfer.KICKER_LOAD_POWER);
                setFeederPower(Constants.Transfer.FEEDER_LOAD_POWER);
            }
        );
    }

    /**
     * @return a command that stops the transfer motor
     */
    public Command getStopCommand() {
        return new InstantCommand(
            () -> {
                kickerMotor.stopMotor();
                feederMotor.stopMotor();
            }
        );
    }
}

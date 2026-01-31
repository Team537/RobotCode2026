package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
    public TalonFX transfer;
    private SparkMax transferNeo;
    private SparkMaxConfig transferConfig;

    //Configuration for the transfer subsystem
    public TransferSubsystem() {
        //transfer = new TalonFX(Constants.Transfer.TRANSFER_MOTOR_ID);
        //transfer.getConfigurator().apply(Configs.TRANSFER_CONFIG);

        transferConfig = new SparkMaxConfig();

        transferConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Transfer.CURRENT_LIMIT)
            .inverted(false);
        transferConfig.encoder
            .positionConversionFactor(Constants.Transfer.ENCODER_FACTOR)
            .velocityConversionFactor(Constants.Transfer.ENCODER_FACTOR / 60.0);
        transferConfig.closedLoop
            .pid(
                Constants.Transfer.KP,
                Constants.Transfer.KI,
                Constants.Transfer.KD
            );
    }
    public void setVelocity(double velocity) {
        //VelocityVoltage velocityRequest = new VelocityVoltage(velocity);

       // transfer.setControl(velocityRequest);

       transferNeo.getClosedLoopController().setSetpoint(1.0, ControlType.kVelocity);
    }

    public Command getSetVelocityCommand(double velocity) {
        return new InstantCommand(
            () -> {
                setVelocity(velocity);
            }
        );
    }

    public Command getLoadCommand() {
       return getSetVelocityCommand(Constants.Transfer.LOAD_SPEED);
    }

    public Command getStopCommand() {
        return getSetVelocityCommand(0.0);
    }
}

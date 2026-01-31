package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
    public TalonFX transfer;

    //Configuration for the transfer subsystem
    public TransferSubsystem() {
        transfer = new TalonFX(Constants.Transfer.TRANSFER_MOTOR_ID);
        transfer.getConfigurator().apply(Configs.TRANSFER_CONFIG);
    }

    public void setVelocity(double velocity) {
        VelocityVoltage velocityRequest = new VelocityVoltage(velocity);

        transfer.setControl(velocityRequest);
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

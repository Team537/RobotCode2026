package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX climber;
    private Servo hook = new Servo(Constants.Climber.HOOK_ID);

    public ClimberSubsystem() {
        climber = new TalonFX(Constants.Climber.CLIMBER_ID);
        climber.getConfigurator().apply(Configs.CLIMBER_CONFIGURATION);
    }

    public double getPosition() {
        return climber.getPosition().getValueAsDouble();
    }

    public void setClimberPosition(double position) {
        PositionVoltage positionRequest = new PositionVoltage(position);
        climber.setControl(positionRequest);
    }

    //Deploy both climber and hook to allow the robot to position itself to climb
    public Command getDeployedPositionCommand() {
        return new InstantCommand(() -> {
            setClimberPosition(Constants.Climber.DEPLOYED_POSITION);
            hook.set(Constants.Climber.HOOK_CLIMB_POSITION);

            if (Math.abs(Constants.Climber.DEPLOYED_POSITION - getPosition()) >= Constants.Climber.CLIMBER_TOLERANCE) {
                setClimberPosition(getPosition());
            }
        }, this);
    }

    //Reset the climber and hook
    public Command getRetractPositionCommand() {
        return new InstantCommand(
            () -> {
                setClimberPosition(Constants.Climber.RETRACTED_POSITION);
                hook.set(Constants.Climber.HOOK_RETRACT_POSITION);
            }, this
        );
    }

    //Lower the climber but keep hook raised allowing for robot to climb
    public Command getClimbPositionCommand() {
        return new InstantCommand(() -> {
            setClimberPosition(Constants.Climber.RETRACTED_POSITION);
            hook.set(Constants.Climber.HOOK_CLIMB_POSITION);
        }, this);
    }
   
}

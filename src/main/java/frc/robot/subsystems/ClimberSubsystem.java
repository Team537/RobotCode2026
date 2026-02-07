package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private Servo climber = new Servo(Constants.Climber.CLIMBER_ID);
    private Servo hook = new Servo(Constants.Climber.HOOK_ID);

    //Deploy both climber and hook to allow the robot to position itself to climb
    public Command getDeployedPositionCommand() {
        return new InstantCommand(() -> {
            climber.set(Constants.Climber.DEPLOYED_POSITION);
            hook.set(Constants.Climber.HOOK_CLIMB_POSITION);
        }, this);
    }

    //Reset the climber and hook
    public Command getRetractPositionCommand() {
        return new InstantCommand(
            () -> {
                climber.set(Constants.Climber.RETRACTED_POSITION);
                hook.set(Constants.Climber.HOOK_RETRACT_POSITION);
            }, this
        );
    }

    //Lower the climber but keep hook raised allowing for robot to climb
    public Command getClimbPositionCommand() {
        return new InstantCommand(() -> {
            climber.set(Constants.Climber.RETRACTED_POSITION);
            hook.set(Constants.Climber.HOOK_CLIMB_POSITION);
        }, this);
    }
   
}

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    // Deploy both climber and hook to allow the robot to position itself to climb
    public Command getDeployedPositionCommand() {
        final double target = Constants.Climber.DEPLOYED_POSITION;
        // Start with the hook retracted. Once the climber reaches the target within
        // tolerance, lower the hook to the climb position and hold the climber.
        java.util.function.BooleanSupplier isFinished = () -> Math.abs(getPosition() - target) <= Constants.Climber.CLIMBER_TOLERANCE;

        return Commands.sequence(
            // initialize: set climber target and ensure hook is retracted
            Commands.runOnce(() -> {
                setClimberPosition(target);
                hook.set(Constants.Climber.HOOK_RETRACT_POSITION);
            }, this),
            // keep commanding climber until within tolerance
            Commands.run(() -> setClimberPosition(target), this).until(isFinished),
            // once within tolerance, lower the hook to the climb position
            Commands.runOnce(() -> hook.set(Constants.Climber.HOOK_CLIMB_POSITION), this),
            // hold the current climber position
            Commands.runOnce(() -> setClimberPosition(getPosition()), this)
        );
    }

    

    // Reset the climber and hook
    public Command getRetractPositionCommand() {
        final double target = Constants.Climber.RETRACTED_POSITION;
        final double hookPos = Constants.Climber.HOOK_RETRACT_POSITION;

        java.util.function.BooleanSupplier isFinished = () -> Math.abs(getPosition() - target) <= Constants.Climber.CLIMBER_TOLERANCE;

        return Commands.sequence(
            Commands.runOnce(() -> {
                setClimberPosition(target);
                hook.set(hookPos);
            }, this),
            Commands.run(() -> setClimberPosition(target), this).until(isFinished),
            Commands.runOnce(() -> setClimberPosition(getPosition()), this)
        );
    }

    // Lower the climber but keep hook raised allowing for robot to climb
    public Command getClimbPositionCommand() {
        final double target = Constants.Climber.RETRACTED_POSITION;
        final double hookPos = Constants.Climber.HOOK_CLIMB_POSITION;
        java.util.function.BooleanSupplier isFinished = () -> Math.abs(getPosition() - target) <= Constants.Climber.CLIMBER_TOLERANCE;

        return Commands.sequence(
            Commands.runOnce(() -> {
                setClimberPosition(target);
                hook.set(hookPos);
            }, this),
            Commands.run(() -> setClimberPosition(target), this).until(isFinished),
            Commands.runOnce(() -> setClimberPosition(getPosition()), this)
        );
    }
   
}

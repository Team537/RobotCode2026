package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX climber;
    private Servo hook = new Servo(Constants.Climber.HOOK_ID);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ClimberSubsystem() {
        climber = new TalonFX(Constants.Climber.CLIMBER_ID);
        StatusCode status = climber.getConfigurator().apply(Configs.CLIMBER_CONFIGURATION);
        
        if (!status.isOK()) {
            DriverStation.reportError(
                "ClimberSubsystem: Failed to configure TalonFX (ID: " + Constants.Climber.CLIMBER_ID + 
                "). Status: " + status.getName() + " - " + status.getDescription(), 
                false
            );
        }
    }

    public double getPosition() {
        return climber.getPosition().getValueAsDouble();
    }

    public void setClimberPosition(double position) {
        positionRequest.Position = position;
        climber.setControl(positionRequest);
    }

    public void stopClimber() {
        climber.setControl(new NeutralOut());
    }

    //Deploy both climber and hook to allow the robot to position itself to climb
    public Command getDeployedPositionCommand() {
        return new FunctionalCommand(
            // Initialize: Set target positions
            () -> {
                setClimberPosition(Constants.Climber.DEPLOYED_POSITION);
                hook.set(Constants.Climber.HOOK_CLIMB_POSITION);
            },
            // Execute: Do nothing, motor controller handles position control
            () -> {},
            // End: Stop the climber and retract hook to safe position if interrupted/timed out
            (interrupted) -> {
                if (interrupted) {
                    stopClimber();
                    hook.set(Constants.Climber.HOOK_RETRACT_POSITION);
                }
                // Otherwise keep holding position (TalonFX continues position control)
            },
            // Stop condition: Check if within tolerance
            () -> Math.abs(Constants.Climber.DEPLOYED_POSITION - getPosition()) < Constants.Climber.CLIMBER_TOLERANCE,
            this
        ).withTimeout(Constants.Climber.DEPLOY_TIMEOUT);
    }

    //Reset the climber and hook
    public Command getRetractPositionCommand() {
        return new FunctionalCommand(
            // Initialize: Set target positions
            () -> {
                setClimberPosition(Constants.Climber.RETRACTED_POSITION);
                hook.set(Constants.Climber.HOOK_RETRACT_POSITION);
            },
            // Execute: Do nothing, motor controller handles position control
            () -> {},
            // End: Stop the climber if interrupted/timed out, otherwise keep holding position
            (interrupted) -> {
                if (interrupted) {
                    stopClimber();
                }
                // Otherwise keep holding position (TalonFX continues position control)
            },
            // Stop condition: Check if within tolerance
            () -> Math.abs(Constants.Climber.RETRACTED_POSITION - getPosition()) < Constants.Climber.CLIMBER_TOLERANCE,
            this
        ).withTimeout(Constants.Climber.RETRACT_TIMEOUT);
    }

    //Lower the climber but keep hook raised allowing for robot to climb
    public Command getClimbPositionCommand() {
        return new FunctionalCommand(
            // Initialize: Set target positions
            () -> {
                setClimberPosition(Constants.Climber.RETRACTED_POSITION);
                hook.set(Constants.Climber.HOOK_CLIMB_POSITION);
            },
            // Execute: Do nothing, motor controller handles position control
            () -> {},
            // End: Stop the climber if interrupted/timed out, otherwise keep holding position
            (interrupted) -> {
                if (interrupted) {
                    stopClimber();
                }
                // Otherwise keep holding position (TalonFX continues position control)
            },
            // Stop condition: Check if within tolerance
            () -> Math.abs(Constants.Climber.RETRACTED_POSITION - getPosition()) < Constants.Climber.CLIMBER_TOLERANCE,
            this
        ).withTimeout(Constants.Climber.CLIMB_TIMEOUT);
    }
   
}

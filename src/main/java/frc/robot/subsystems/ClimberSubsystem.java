package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX climber;

    public ClimberSubsystem() {
        climber = new TalonFX(Constants.Climber.CLIMBER_ID);
        climber.getConfigurator().apply(Configs.CLIMBER_CONFIG);
    }

    public void setClimberAngle(Rotation2d angle) {
        PositionVoltage angleRequest;

        angleRequest = new PositionVoltage(angle.getRadians());
        climber.setControl(angleRequest);
    }

    public Command goToDeployAngleCommand() {
        return new RunCommand(
            () -> {
                setClimberAngle(Constants.Climber.DEPLOYED_WINCH_ROTATIONS);
            },
            this
        );
    }

    public Command climbCommand() {
        return new RunCommand(
            () -> {
                setClimberAngle(Constants.Climber.CLIMB_WINCH_ROTATIONS);
            },
            this
        );
    }

    public boolean isAtClimbAngle() {
        double current_pos = climber.getPosition().getValueAsDouble();

        return Math.abs(current_pos - Constants.Climber.CLIMB_WINCH_ROTATIONS.getRotations()) < Constants.Climber.CLIMBER_ANGLE_TOLERANCE.getRotations();
    }

    public boolean isAtDeployAngle() {
        double current_pos = climber.getPosition().getValueAsDouble();

        return Math.abs(current_pos - Constants.Climber.DEPLOYED_WINCH_ROTATIONS.getRotations()) < Constants.Climber.CLIMBER_ANGLE_TOLERANCE.getRotations();
    }
}

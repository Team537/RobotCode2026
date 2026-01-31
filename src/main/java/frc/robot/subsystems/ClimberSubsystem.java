package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX climber;
    private SparkMax climberNeo;
    private SparkMaxConfig climberConfig;

    //Configuration
    public ClimberSubsystem() {
        //climber = new TalonFX(Constants.Climber.CLIMBER_ID);
        //climber.getConfigurator().apply(Configs.CLIMBER_CONFIG);

        climberConfig = new SparkMaxConfig();
        climberConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(false);

        climberConfig.encoder
            .positionConversionFactor(1.0/Constants.Climber.CLIMBER_ANGLE_TO_MOTOR_ANGLE);

        climberConfig.closedLoop
            .pid(
                Constants.Climber.KP,
                Constants.Climber.KI,
                Constants.Climber.KD
            );
    }

    //Converts the angle to radians allowing for it to be changed from Rotation2d to double
    public void setClimberAngle(Rotation2d angle) {
        //PositionVoltage angleRequest;

        //angleRequest = new PositionVoltage(angle.getRadians());
        //climber.setControl(angleRequest);

        climberNeo.getClosedLoopController().setSetpoint(angle.getRadians(), ControlType.kPosition);
    }

    //Sets the robot up for climbing
    public Command goToDeployAngleCommand() {
        return new RunCommand(
            () -> {
                setClimberAngle(Constants.Climber.DEPLOYED_WINCH_ROTATIONS);
            },
            this
        );
    }

    //Makes the robot start climbing
    public Command climbCommand() {
        return new RunCommand(
            () -> {
                setClimberAngle(Constants.Climber.CLIMB_WINCH_ROTATIONS);
            },
            this
        );
    }


    //Checks if the climber is at the correct angle for climbing
    public boolean isAtClimbAngle() {
        double current_pos = climberNeo.getEncoder().getPosition();

        return Math.abs(current_pos - Constants.Climber.CLIMB_WINCH_ROTATIONS.getRotations()) < Constants.Climber.CLIMBER_ANGLE_TOLERANCE.getRotations();
    }


    //Checks if the climber is at the correct angle deployed
    public boolean isAtDeployAngle() {
        double current_pos = climberNeo.getEncoder().getPosition();

        return Math.abs(current_pos - Constants.Climber.DEPLOYED_WINCH_ROTATIONS.getRotations()) < Constants.Climber.CLIMBER_ANGLE_TOLERANCE.getRotations();
    }
}

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {
    private TalonFX intakeRoller;
    private SparkMax intakeRollerNeo;
    private SparkMaxConfig intakeRollerConfig;
    
    //Configuration (Talon)
    public IntakeRollerSubsystem() {
        //intakeRoller = new TalonFX(Constants.Intake.INTAKE_LEAD_ID);
        //intakeRoller.getConfigurator().apply(Configs.Intake.INTAKE_CONFIGURATION);

        //Configuration (neo)
        intakeRollerConfig = new SparkMaxConfig();
        intakeRollerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Intake.CURRENT_LIMIT)
            .inverted(Constants.Intake.MOTOR_INVERTED_NEO);
        intakeRollerConfig.encoder
            .positionConversionFactor(Constants.Intake.ENCODER_FACTOR)
            .velocityConversionFactor(Constants.Intake.ENCODER_FACTOR / 60.0);
        intakeRollerConfig.closedLoop
            .pid(
                Constants.Intake.KP,
                Constants.Intake.KI,
                Constants.Intake.KD
            );

    }

    //Sets the velocity of the intake roller
    public void setVelocity(double velocity) {
        //VelocityVoltage velocityRequest = new VelocityVoltage(velocity);
        intakeRollerNeo.getClosedLoopController().setSetpoint(1.0, ControlType.kVelocity);
    }

    //Runs the command for the intake roller at a given velocity
    public Command getVelocityCommand(Supplier<Double> velocitySupplier) {
        return new InstantCommand(
            () -> {
                setVelocity(velocitySupplier.get());
            },
            this
        );
    }
}

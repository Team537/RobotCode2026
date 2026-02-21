package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {
    private TalonFX intakeRoller;
    
    //Configuration (Talon)
    public IntakeRollerSubsystem() {
        intakeRoller = new TalonFX(Constants.Intake.INTAKE_ID);
        intakeRoller.getConfigurator().apply(Configs.Intake.INTAKE_CONFIGURATION);
    }

    public Command getIntakeCommand() {
        return new RunCommand(() -> intakeRoller.set(1.0),this);
    }

    public Command getStopCommand() {
        return new RunCommand(() -> intakeRoller.stopMotor(),this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Speed (Radians / Second)",intakeRoller.getVelocity().getValueAsDouble());
    }

}

package frc.robot.subsystems;

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

    // --------------------------------------------------------------------
    // Hardware
    // --------------------------------------------------------------------

    private final TalonFX intakeRoller;

    // --------------------------------------------------------------------
    // Construction / Configuration
    // --------------------------------------------------------------------

    public IntakeRollerSubsystem() {
        intakeRoller = new TalonFX(Constants.Intake.INTAKE_ID);
        intakeRoller.getConfigurator().apply(Configs.Intake.INTAKE_CONFIGURATION);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Intake Roller Velocity",
            getVelocity()
        );
    }

    // --------------------------------------------------------------------
    // Control
    // --------------------------------------------------------------------

    public double getVelocity() {
        return intakeRoller.getVelocity().getValueAsDouble();
    }

    // --------------------------------------------------------------------
    // Commands
    // --------------------------------------------------------------------
    public Command getIntakeCommand() {
        return new RunCommand(() -> intakeRoller.set(Constants.Intake.INTAKE_POWER),this);
    }

    public Command getStopCommand() {
        return new RunCommand(() -> intakeRoller.stopMotor(),this);
    }
}
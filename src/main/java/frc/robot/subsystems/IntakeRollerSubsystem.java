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
        intakeRoller = new TalonFX(Constants.IntakeRoller.INTAKE_ID, Constants.CANIVORE_LOOP_NAME);
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
        return new InstantCommand(() -> intakeRoller.set(Constants.IntakeRoller.INTAKE_POWER),this).withName("RunIntake");
    }

    public Command getStopCommand() {
        return new InstantCommand(() -> intakeRoller.stopMotor(),this).withName("StopIntake");
    }
}
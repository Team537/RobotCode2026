package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {
    private TalonFX intakeRoller;

    public IntakeRollerSubsystem() {
        intakeRoller = new TalonFX(Constants.Intake.INTAKE_LEAD_ID);
        intakeRoller.getConfigurator().apply(Configs.Intake.INTAKE_CONFIGURATION);
    }

    public void setVelocity(double velocity) {
        VelocityVoltage velocityRequest = new VelocityVoltage(velocity);
        intakeRoller.setControl(velocityRequest);
    }

    public Command getVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
            () -> {
                setVelocity(velocitySupplier.get());
            },
            this
        );
    }
}

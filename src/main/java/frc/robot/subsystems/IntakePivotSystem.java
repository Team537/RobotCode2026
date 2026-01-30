package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakePivotSystem extends SubsystemBase {
    public TalonFX Intake;

    //Configuration
    public IntakePivotSystem() {
        Intake = new TalonFX(Constants.IntakePivot.INTAKE_ID);
        Intake.getConfigurator().apply(Configs.IntakePivot.INTAKE_PIVOT_CONFIGURATION);
    }

    //Sets the angle for the intake subsystem
    public void setIntakeAngle(Rotation2d angle) {
        PositionVoltage angleRequest;

        angleRequest = new PositionVoltage(angle.getRadians());
        
        Intake.setControl(angleRequest);
    }

    //Returns the angle as a double
    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(Intake.getPosition().getValueAsDouble());
    }

    
    //Lowers the intake
    public Command setIntakeAngleCommand(Supplier<Rotation2d> angleSupplier){
        return new RunCommand(
            () -> {
                setIntakeAngle(angleSupplier.get());

                //Prevents the intake from suprassing the maximum angle (pushing down on the floor)
                if (getAngle().getRadians() > Constants.IntakePivot.INTAKE_MAX_ANGLE.getRadians()) {
                    setIntakeAngle(Constants.IntakePivot.INTAKE_MAX_ANGLE);
                }
            }
        );
    }

    //Raises the intake to the start position
    public Command raiseIntakeCommand() {
        return new RunCommand(
            () -> {
                setIntakeAngle(Constants.IntakePivot.INTAKE_START_POS);
            }
        );
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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

        //Prevents the intake from suprassing the maximum angle (going past its start pos)        
        if (angle.getRadians() > Constants.IntakePivot.INTAKE_MAX_ANGLE.getRadians()) {
            angle = Constants.IntakePivot.INTAKE_MAX_ANGLE;
        }
                
        //Prevents the intake from surpassing its minimum angle (going too far down)
        if (angle.getRadians() > Constants.IntakePivot.INTAKE_MIN_ANGLE.getRadians()) {
            angle = Constants.IntakePivot.INTAKE_MIN_ANGLE;
        }

        PositionVoltage angleRequest;

        angleRequest = new PositionVoltage(angle.getRadians());

        
        Intake.setControl(angleRequest);
    }

    //Returns the angle as a double
    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(Intake.getPosition().getValueAsDouble());
    }

    
    //Command for setting hte intake angle
    public Command setIntakeAngleCommand(Rotation2d angle){
        return new RunCommand(
            () -> {
                setIntakeAngle(angle);
            }
        ).until(() -> {
            Rotation2d difference = getAngle().minus(angle);        
            return Math.abs(difference.getRadians()) < Constants.IntakePivot.INTAKE_TOLERANCE_ANGLE.getRadians();
        });
    }

    //Raises the intake to the start position
    public Command raiseIntakeCommand() {
        return setIntakeAngleCommand(Constants.IntakePivot.INTAKE_START_POS);
    }

    //Deploys intake 
    public Command deployIntakeCommand() {
        return setIntakeAngleCommand(Constants.IntakePivot.INTAKE_DEPLOYED_ANGLE);
    }
}

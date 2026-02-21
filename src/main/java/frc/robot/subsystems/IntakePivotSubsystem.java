package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakePivotSubsystem extends SubsystemBase {
    public TalonFX intake;
    private final PositionVoltage angleRequest = new PositionVoltage(0);
    private Rotation2d targetAngle = Constants.IntakePivot.INTAKE_START_POS;
    private boolean commandActive = false;

    //Configuration
    public IntakePivotSubsystem() {
        intake = new TalonFX(Constants.IntakePivot.INTAKE_ID);
        StatusCode status = intake.getConfigurator().apply(Configs.IntakePivot.INTAKE_PIVOT_CONFIGURATION);
        
        if (!status.isOK()) {
            DriverStation.reportError(
                "IntakePivotSubsystem: Failed to configure TalonFX (ID: " + Constants.IntakePivot.INTAKE_ID + 
                "). Status: " + status.getName() + " - " + status.getDescription(), 
                false
            );
        } else {
            System.out.println("IntakePivotSubsystem: TalonFX configured successfully");
        }
        
        intake.setPosition(Constants.IntakePivot.INTAKE_START_POS.getRadians());
        System.out.println("IntakePivotSubsystem: Initial position set to " + Constants.IntakePivot.INTAKE_START_POS.getDegrees() + " degrees");
    }

    //Sets the angle for the intake subsystem
    public void setIntakeAngle(Rotation2d angle) {
        targetAngle = angle;

        //Prevents the intake from surpassing the maximum angle (going past its start pos)        
        if (angle.getRadians() > Constants.IntakePivot.INTAKE_MAX_ANGLE.getRadians()) {
            angle = Constants.IntakePivot.INTAKE_MAX_ANGLE;
            System.out.println("IntakePivot: Clamped to MAX angle: " + angle.getDegrees());
        }
                
        //Prevents the intake from surpassing its minimum angle (going too far down)
        if (angle.getRadians() < Constants.IntakePivot.INTAKE_MIN_ANGLE.getRadians()) {
            angle = Constants.IntakePivot.INTAKE_MIN_ANGLE;
            System.out.println("IntakePivot: Clamped to MIN angle: " + angle.getDegrees());
        }

        angleRequest.Position = angle.getRadians();
        intake.setControl(angleRequest);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivot/CurrentAngle", getAngle().getDegrees());
        SmartDashboard.putNumber("IntakePivot/TargetAngle", targetAngle.getDegrees());
        SmartDashboard.putNumber("IntakePivot/Error", getAngle().minus(targetAngle).getDegrees());
        SmartDashboard.putBoolean("IntakePivot/CommandActive", commandActive);
        SmartDashboard.putNumber("IntakePivot/MotorVoltage", intake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("IntakePivot/MotorCurrent", intake.getSupplyCurrent().getValueAsDouble());
    }

    //Returns the angle as a double
    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(intake.getPosition().getValueAsDouble());
    }
    
    //Command for setting hte intake angle
    public Command setIntakeAngleCommand(Rotation2d angle){
        return new RunCommand(
            () -> {
                setIntakeAngle(angle);
            },
            this
        )
        .beforeStarting(() -> {
            commandActive = true;
            System.out.println("IntakePivot: Starting command to angle " + angle.getDegrees() + " degrees");
            System.out.println("IntakePivot: Current angle is " + getAngle().getDegrees() + " degrees");
        })
        .until(() -> {
            Rotation2d difference = getAngle().minus(angle);
            boolean finished = Math.abs(difference.getRadians()) < Constants.IntakePivot.INTAKE_TOLERANCE_ANGLE.getRadians();
            if (finished) {
                System.out.println("IntakePivot: Reached target angle " + angle.getDegrees() + " degrees");
            }
            return finished;
        })
        .finallyDo(() -> {
            commandActive = false;
            intake.stopMotor();
            System.out.println("IntakePivot: Command finished");
        });
    }

    //Raises the intake to the start position
    public Command raiseIntakeCommand() {
        return setIntakeAngleCommand(Constants.IntakePivot.INTAKE_RAISED_ANGLE)
            .withName("RaiseIntake");
    }

    //Deploys intake 
    public Command deployIntakeCommand() {
        return setIntakeAngleCommand(Constants.IntakePivot.INTAKE_DEPLOYED_ANGLE)
            .withName("DeployIntake");
    }
}

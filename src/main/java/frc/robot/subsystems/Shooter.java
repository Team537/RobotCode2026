package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private TalonFX shooter;
    private SparkMax shooterNeo;
    private SparkMaxConfig shooterConfig;

     private Supplier<Pose2d> robotPoseSupplier = () -> Pose2d.kZero;

     //Shooter configuration
    public Shooter(){
        //shooter = new TalonFX(Constants.Shooter.SHOOTER_ID);
        //shooter.getConfigurator().apply(Configs.Shooter.SHOOTER_CONFIGURATION);

        shooterConfig = new SparkMaxConfig();

        shooterConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Shooter.CURRENT_LIMIT)
            .inverted(false);

        shooterConfig.encoder
            .positionConversionFactor(Constants.Shooter.ENCOER_FACTOR)
            .velocityConversionFactor(Constants.Shooter.ENCOER_FACTOR / 60.0);

        shooterConfig.closedLoop
            .pid(
                Constants.Shooter.KP,
                Constants.Shooter.KI,
                Constants.Shooter.KD
                );

    }

    //Used for setting the velocity of the shooter wheel(s)
    public void setVelocity(double velocity) {
        //VelocityVoltage velocityRequest = new VelocityVoltage(velocity);
        //shooter.setControl(velocityRequest);

        shooterNeo.getClosedLoopController().setSetpoint(1.0, ControlType.kVelocity);
    }

    //Sets up the supplier for the robot pose, used in the pid to determine the shooting power
    public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier){
        this.robotPoseSupplier = robotPoseSupplier;
    }

    //Runs the command for setting the shooter velocity
    public Command setVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
            () -> {
                setVelocity(velocitySupplier.get());
            },
            this
        );
    }

    //Sets the target for shooting, this is for determining the power
    public Command getTargetCommand(Supplier<Translation3d> targetTranslationSupplier) {
        return new RunCommand(
            () -> {
                Translation3d targetTranslation3d = targetTranslationSupplier.get();
                Translation2d targetTranslation = new Translation2d(targetTranslation3d.getX(), targetTranslation3d.getY());

                Translation2d robotTranslation = targetTranslationSupplier.get().toTranslation2d();

                Translation2d displacement = targetTranslation.minus(robotTranslation);
                
                setVelocity(displacement.getNorm());
            },
            this
        );
    }
}

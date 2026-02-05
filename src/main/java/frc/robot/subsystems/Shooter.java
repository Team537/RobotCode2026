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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    private TalonFX shooter;

    private boolean atTarget = false;

    //Shooter configuration
    public Shooter(){
        shooter = new TalonFX(Constants.Shooter.SHOOTER_ID);
        shooter.getConfigurator().apply(Configs.Shooter.SHOOTER_CONFIGURATION);

    }

    //Used for setting the velocity of the shooter wheel(s)
    public void setVelocity(double velocity) {
        VelocityVoltage velocityRequest = new VelocityVoltage(velocity);
        shooter.setControl(velocityRequest);
    }

    public double getVelocity() {
        return shooter.getVelocity().getValueAsDouble();
    }

    //Runs the command for setting the shooter velocity
    public Command setVelocityCommand(Supplier<Double> velocitySupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setVelocity(velocitySupplier.get());
                atTarget = Math.abs(velocitySupplier.get() - getVelocity()) < Constants.Shooter.TOLERANCE;
            },
            interrupted -> atTarget = false,
            () -> false,
            this
        );
    }

    //Sets the target for shooting, this is for determining the power
    public Command getTargetCommand(Supplier<Translation3d> targetTranslationSupplier, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier ) {
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

    public boolean atTarget() {
        return atTarget;
    }

}

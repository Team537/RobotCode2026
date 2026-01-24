package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private final TalonFX shooter;
     private Supplier<Pose2d> robotPoseSupplier = () -> Pose2d.kZero;

    public Shooter(){
        shooter = new TalonFX(Constants.Shooter.SHOOTER_ID);
        shooter.getConfigurator().apply(Configs.Shooter.SHOOTER_CONFIGURATION);
    }

    public void setVelocity(double velocity) {
        VelocityVoltage velocityRequest = new VelocityVoltage(velocity);
        shooter.setControl(velocityRequest);
    }

    public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier){
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public Command setVelocityCommand(Supplier<Double> velocitySupplier) {
        return new RunCommand(
            () -> {
                setVelocity(velocitySupplier.get());
            },
            this
        );
    }

    public Command getTargetCommand(Supplier<Translation3d> targetTranslationSupplier) {
        return new RunCommand(
            () -> {
                Translation3d targetTranslation3d = targetTranslationSupplier.get();
                Translation2d targetTranslation = new Translation2d(targetTranslation3d.getX(), targetTranslation3d.getY());

                Translation2d robotTranslation = targetTranslationSupplier.get().toTranslation2d();

                Translation2d displacement = targetTranslation.minus(robotTranslation);
                Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
            }
        );
    }
}

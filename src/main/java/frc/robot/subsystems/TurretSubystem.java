package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.cscore.raw.RawSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.Turret;;

public class TurretSubystem extends SubsystemBase {
    private final TalonFX turret;
    private Supplier<Pose2d> robotPoseSupplier = () -> Pose2d.kZero;

    public TurretSubystem(){
        
        turret = new TalonFX(Constants.Turret.TURRET_ID);
        turret.getConfigurator().apply(Configs.TURRET_CONFIG);
        turret.setPosition(Constants.Turret.START_POS.getRadians());
    }

    public void setTurretAngle(Rotation2d angle) {
        PositionVoltage angleRequest;

        angleRequest = new PositionVoltage(angle.getRadians());
        turret.setControl(angleRequest);
    }
    public void setTurretAngleFieldRelative(Rotation2d angle) {
        Rotation2d robotHeading = robotPoseSupplier.get().getRotation();

        setTurretAngle(angle.minus(robotHeading));

    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(turret.getPosition().getValueAsDouble());
    }

    public Rotation2d getAngleFieldRelative(Rotation2d angle) {
        Rotation2d robotHeading = robotPoseSupplier.get().getRotation();

        return getAngle().plus(robotHeading);
    }

    public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier){
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public Command getAngleCommand(Supplier<Rotation2d> angleSupplier){
        return run(
            () -> {
                setTurretAngle(angleSupplier.get());
            }
        );
    }

    public Command getTargetCommand(Supplier<Translation3d> targetTranslationSupplier) {
        return run(
            ()-> {
                Translation3d targetTranslation3d = targetTranslationSupplier.get();
                Translation2d targetTranslation = new Translation2d(targetTranslation3d.getX(),targetTranslation3d.getY());

                Translation2d robotTranslation = targetTranslationSupplier.get().toTranslation2d();

                Translation2d displacement = targetTranslation.minus(robotTranslation);
                Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());

                setTurretAngleFieldRelative(angle);
            }
        );
    }
}


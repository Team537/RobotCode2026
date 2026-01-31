package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubystem extends SubsystemBase {
    private TalonFX turret;

    private SparkMax turretNeo;
    private SparkMaxConfig turretConfig;

    private Supplier<Pose2d> robotPoseSupplier = () -> Pose2d.kZero;

    //Turret Configuration
    public TurretSubystem(){
        
        //turret = new TalonFX(Constants.Turret.TURRET_ID);
        //turret.getConfigurator().apply(Configs.TURRET_CONFIG);
        //turret.setPosition(Constants.Turret.START_POS.getRadians());

        turretConfig = new SparkMaxConfig();

        turretConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Turret.TURRET_MOTOR_CURRENT_LIMIT)
            .inverted(false);

        turretConfig.encoder
            .positionConversionFactor(Constants.Turret.ENCODER_FACTOR)
            .velocityConversionFactor(Constants.Turret.ENCODER_FACTOR / 60.0);
        
        turretConfig.closedLoop
            .pid(
                Constants.Turret.TURRET_KP,
                Constants.Turret.TURRET_KI,
                Constants.Turret.TURRET_KD
            );
    }

    //Converts the angle to radians allowing for it to be changed from Rotation2d to double
    public void setTurretAngle(Rotation2d angle) {
        //PositionVoltage angleRequest;

        //angleRequest = new PositionVoltage(angle.getRadians());
        //turret.setControl(angleRequest);

        turretNeo.getClosedLoopController().setSetpoint(angle.getRadians(), com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    //Sets the turret angle relative to the field
    public void setTurretAngleFieldRelative(Rotation2d angle) {
        Rotation2d robotHeading = robotPoseSupplier.get().getRotation();

        setTurretAngle(angle.minus(robotHeading));

    }

    //Finds the current angle of the turret
    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(turretNeo.getEncoder().getPosition());
    }

    //Finds the current angle of the turret relative to the field
    public Rotation2d getAngleFieldRelative(Rotation2d angle) {
        Rotation2d robotHeading = robotPoseSupplier.get().getRotation();

        return getAngle().plus(robotHeading);
    }

    //Sets up the supplier for the robot pose
    public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier){
        this.robotPoseSupplier = robotPoseSupplier;
    }

    //Runs the command for setting the turret angle
    public Command getAngleCommand(Supplier<Rotation2d> angleSupplier){
        return new RunCommand(
            () -> {
                setTurretAngle(angleSupplier.get());
            },
            this
        );
    }

    //Sets the target for shooting
    public Command getTargetCommand(Supplier<Translation3d> targetTranslationSupplier) {
        return new RunCommand(
            ()-> {
                Translation3d targetTranslation3d = targetTranslationSupplier.get();
                Translation2d targetTranslation = new Translation2d(targetTranslation3d.getX(),targetTranslation3d.getY());

                Translation2d robotTranslation = targetTranslationSupplier.get().toTranslation2d();

                Translation2d displacement = targetTranslation.minus(robotTranslation);
                Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());

                setTurretAngleFieldRelative(angle);
            },
            this
        );
    }
}


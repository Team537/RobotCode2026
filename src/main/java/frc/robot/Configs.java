package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    // Deployed by GradleRIO FileArtifact in build.gradle
    public static final String DEPLOY_METADATA_PATH = "/home/lvuser/537metadata.txt";

    // NetworkTables locations (Elastic can display String topics easily)
    public static final String NT_TABLE = "Build537";
    public static final String NT_RAW_KEY = "DeployMetadataRaw";   // full file contents
    public static final String NT_STATUS_KEY = "DeployMetadataStatus"; // diag/status text
    public static final String NT_PREFIX_PARSED = "DeployMetadata/";   // per-field entries
    
    public static final TalonFXConfiguration TURRET_CONFIG = new TalonFXConfiguration();

    static {

        TURRET_CONFIG
            .Feedback.SensorToMechanismRatio = Constants.Turret.ENCODER_FACTOR;
        
        TURRET_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Turret.TURRET_MOTOR_CURRENT_LIMIT;

        TURRET_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Turret.CURRENT_LOWER_LIMIT;

        TURRET_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Turret.CURRENT_LOWER_TIME;

        TURRET_CONFIG
            .MotorOutput.Inverted = Constants.Turret.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        TURRET_CONFIG
            .Slot0
                .kP = Constants.Turret.TURRET_KP;
                
        TURRET_CONFIG
            .Slot0
                .kI = Constants.Turret.TURRET_KI;

        TURRET_CONFIG
            .Slot0
                .kD = Constants.Turret.TURRET_KD;

        TURRET_CONFIG
            .MotorOutput
                .NeutralMode = NeutralModeValue.Brake;
    }



    public static final class Shooter {
        public static final TalonFXConfiguration SHOOTER_CONFIGURATION = new TalonFXConfiguration();

        static {

            SHOOTER_CONFIGURATION
                .Feedback.SensorToMechanismRatio = Constants.Shooter.ENCOER_FACTOR;

            SHOOTER_CONFIGURATION
                .CurrentLimits
                    .SupplyCurrentLimit = Constants.Shooter.CURRENT_LIMIT;
            
            SHOOTER_CONFIGURATION
                .CurrentLimits
                    .SupplyCurrentLimit = Constants.Shooter.CURRENT_LOWER_TIME;

            SHOOTER_CONFIGURATION
                .CurrentLimits
                    .SupplyCurrentLimit = Constants.Shooter.CURRENT_LOWER_LIMIT;

            SHOOTER_CONFIGURATION
                .Slot0
                    .kI = Constants.Shooter.KI;

            SHOOTER_CONFIGURATION
                .Slot0
                    .kP = Constants.Shooter.KP;

            SHOOTER_CONFIGURATION
                .Slot0
                    .kD = Constants.Shooter.KD;

            SHOOTER_CONFIGURATION
                .MotorOutput
                    .NeutralMode = NeutralModeValue.Brake;
        }
    }

    public static final TalonFXConfiguration TRANSFER_CONFIG = new TalonFXConfiguration();

    static {
        TRANSFER_CONFIG
            .Feedback.SensorToMechanismRatio = Constants.Transfer.ENCODER_FACTOR;

        TRANSFER_CONFIG
            .Slot0
                .kP = Constants.Transfer.KP;

        TRANSFER_CONFIG
            .Slot0
                .kI = Constants.Transfer.KI;

        TRANSFER_CONFIG
            .Slot0
                .kD = Constants.Transfer.KD;

        TRANSFER_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Transfer.CURRENT_LIMIT;
        
        TRANSFER_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Transfer.CURRENT_LOWER_LIMIT;

        TRANSFER_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Transfer.CURRENT_LOWER_TIME;

        TRANSFER_CONFIG
            .MotorOutput
                .Inverted = Constants.Transfer.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        TRANSFER_CONFIG
            .MotorOutput
                .NeutralMode = NeutralModeValue.Brake;
    }
}

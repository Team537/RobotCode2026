package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
    public static final CANcoderConfiguration PITCH_CANCODER_CONFIG = new CANcoderConfiguration();

    static {

        TURRET_CONFIG
            .Feedback.SensorToMechanismRatio = Constants.Turret.ENCODER_FACTOR;
        
        TURRET_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Turret.TURRET_MOTOR_CURRENT_LIMIT;

        TURRET_CONFIG
            .CurrentLimits
                .SupplyCurrentLowerLimit = Constants.Turret.CURRENT_LOWER_LIMIT;

        TURRET_CONFIG
            .CurrentLimits
                .SupplyCurrentLowerTime = Constants.Turret.CURRENT_LOWER_TIME;

        TURRET_CONFIG
            .MotorOutput.Inverted = Constants.Turret.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        TURRET_CONFIG
            .Slot0
                .kP = Constants.Turret.KP;
                
        TURRET_CONFIG
            .Slot0
                .kI = Constants.Turret.KI;

        TURRET_CONFIG
            .Slot0
                .kD = Constants.Turret.KD;

        TURRET_CONFIG
            .Slot0
                .kS = Constants.Turret.KS;     

        TURRET_CONFIG
            .Slot0
                .kV = Constants.Turret.KV;
        
        TURRET_CONFIG
            .Slot0
                .kA = Constants.Turret.KA;

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
                    .SupplyCurrentLowerTime = Constants.Shooter.CURRENT_LOWER_TIME;

            SHOOTER_CONFIGURATION
                .CurrentLimits
                    .SupplyCurrentLowerLimit = Constants.Shooter.CURRENT_LOWER_LIMIT;

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
                .Slot0
                    .kS = Constants.Shooter.KS;

            SHOOTER_CONFIGURATION
                .Slot0
                    .kV = Constants.Shooter.KV;

            SHOOTER_CONFIGURATION
                .Slot0
                    .kA = Constants.Shooter.KA;

            SHOOTER_CONFIGURATION
                .MotorOutput
                    .Inverted = Constants.Shooter.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            SHOOTER_CONFIGURATION
                .MotorOutput
                    .NeutralMode = NeutralModeValue.Coast;
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
            .Slot0
                .kS = Constants.Transfer.KS;

        TRANSFER_CONFIG
            .Slot0
                .kV = Constants.Transfer.KV;

        TRANSFER_CONFIG
            .Slot0
                .kA = Constants.Transfer.KA;

        TRANSFER_CONFIG
            .CurrentLimits
                .SupplyCurrentLimit = Constants.Transfer.CURRENT_LIMIT;
        
        TRANSFER_CONFIG
            .CurrentLimits
                .SupplyCurrentLowerLimit = Constants.Transfer.CURRENT_LOWER_LIMIT;

        TRANSFER_CONFIG
            .CurrentLimits
                .SupplyCurrentLowerTime = Constants.Transfer.CURRENT_LOWER_TIME;

        TRANSFER_CONFIG
            .MotorOutput
                .Inverted = Constants.Transfer.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        TRANSFER_CONFIG
            .MotorOutput
                .NeutralMode = NeutralModeValue.Brake;
    }

    public static class Intake {
        public static TalonFXConfiguration INTAKE_CONFIGURATION = new TalonFXConfiguration();

        static {
            INTAKE_CONFIGURATION
                .Feedback.SensorToMechanismRatio = Constants.Intake.ENCODER_FACTOR;
            
            INTAKE_CONFIGURATION
                .CurrentLimits
                    .SupplyCurrentLimit = Constants.Intake.CURRENT_LIMIT;

            INTAKE_CONFIGURATION
                .CurrentLimits
                    .SupplyCurrentLowerLimit = Constants.Intake.CURRENT_LOWER_LIMIT;

            INTAKE_CONFIGURATION
                .CurrentLimits 
                    .SupplyCurrentLowerTime = Constants.Intake.CURRENT_LOWER_TIME;

            INTAKE_CONFIGURATION
                .MotorOutput
                    .NeutralMode = NeutralModeValue.Coast;

            INTAKE_CONFIGURATION
                .MotorOutput
                    .Inverted = Constants.Intake.MOTOR_INVERTED;
        }


        
    }

    public static class IntakePivot {
        public static TalonFXConfiguration INTAKE_PIVOT_CONFIGURATION = new TalonFXConfiguration();

            static {
                INTAKE_PIVOT_CONFIGURATION
                    .Feedback.RotorToSensorRatio = Constants.IntakePivot.ROTOR_TO_SENSOR_RATIO;
                INTAKE_PIVOT_CONFIGURATION
                    .Feedback.SensorToMechanismRatio = Constants.IntakePivot.SENSOR_TO_MECHANISM_RATIO;
                INTAKE_PIVOT_CONFIGURATION
                    .Feedback.FeedbackRemoteSensorID = Constants.IntakePivot.CANCODER_ID;
                INTAKE_PIVOT_CONFIGURATION
                    .Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                
                INTAKE_PIVOT_CONFIGURATION
                    .CurrentLimits
                        .SupplyCurrentLimit = Constants.IntakePivot.CURRENT_LIMIT;

                INTAKE_PIVOT_CONFIGURATION
                    .CurrentLimits
                        .SupplyCurrentLowerLimit = Constants.IntakePivot.CURRENT_LOWER_LIMIT;

                INTAKE_PIVOT_CONFIGURATION
                    .CurrentLimits
                        .SupplyCurrentLowerTime = Constants.IntakePivot.CURRENT_LOWER_TIME;

                INTAKE_PIVOT_CONFIGURATION
                    .Slot0
                        .kP = Constants.IntakePivot.KP;

                INTAKE_PIVOT_CONFIGURATION
                    .Slot0
                        .kI = Constants.IntakePivot.KI;

                INTAKE_PIVOT_CONFIGURATION
                    .Slot0
                        .kD = Constants.IntakePivot.KD;

                INTAKE_PIVOT_CONFIGURATION
                    .Slot0
                        .kS = Constants.IntakePivot.KS;

                INTAKE_PIVOT_CONFIGURATION
                    .Slot0
                        .kV = Constants.IntakePivot.KV;

                INTAKE_PIVOT_CONFIGURATION
                    .Slot0
                        .kA = Constants.IntakePivot.KA;

                INTAKE_PIVOT_CONFIGURATION
                    .MotorOutput
                        .NeutralMode = NeutralModeValue.Brake;

                INTAKE_PIVOT_CONFIGURATION
                    .MotorOutput
                        .Inverted = Constants.IntakePivot.MOTOR_INVERTED;

            }
        }
}

package frc.robot;

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

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    // Deployed by GradleRIO FileArtifact in build.gradle
    public static final String DEPLOY_METADATA_PATH = "/home/lvuser/537metadata.txt";

    // NetworkTables locations (Elastic can display String topics easily)
    public static final String NT_TABLE = "Build537";
    public static final String NT_RAW_KEY = "DeployMetadataRaw";   // full file contents
    public static final String NT_STATUS_KEY = "DeployMetadataStatus"; // diag/status text
    public static final String NT_PREFIX_PARSED = "DeployMetadata/";   // per-field entries

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
                .Inverted = Constants.Transfer.MOTOR_INVERTED;
        
        TRANSFER_CONFIG
            .MotorOutput
                .NeutralMode = NeutralModeValue.Brake;
    }
}

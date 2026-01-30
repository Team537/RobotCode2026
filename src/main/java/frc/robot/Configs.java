package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Configs {
    // Deployed by GradleRIO FileArtifact in build.gradle
    public static final String DEPLOY_METADATA_PATH = "/home/lvuser/537metadata.txt";

    // NetworkTables locations (Elastic can display String topics easily)
    public static final String NT_TABLE = "Build537";
    public static final String NT_RAW_KEY = "DeployMetadataRaw";   // full file contents
    public static final String NT_STATUS_KEY = "DeployMetadataStatus"; // diag/status text
    public static final String NT_PREFIX_PARSED = "DeployMetadata/";   // per-field entries

    public static final TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();

    static {
        CLIMBER_CONFIG
            .Feedback.SensorToMechanismRatio = Constants.Climber.CLIMBER_ANGLE_TO_MOTOR_ANGLE;

        CLIMBER_CONFIG
            .Slot0
                .kP = Constants.Climber.KP;
        CLIMBER_CONFIG
            .Slot0
                .kI = Constants.Climber.KI;
        CLIMBER_CONFIG
            .Slot0
                .kD = Constants.Climber.KD;
    }
}

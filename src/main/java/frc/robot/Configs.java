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

    public static final TalonFXConfiguration CLIMBER_CONFIGURATION = new TalonFXConfiguration();

    static {

        CLIMBER_CONFIGURATION
            .Feedback.SensorToMechanismRatio = Constants.Climber.ENCODER_FACTOR;

        CLIMBER_CONFIGURATION
            .CurrentLimits.SupplyCurrentLimit = Constants.Climber.MOTOR_LIMIT;

        CLIMBER_CONFIGURATION
            .Slot0.kP = Constants.Climber.KP;

        CLIMBER_CONFIGURATION
            .Slot0.kI = Constants.Climber.KI;

        CLIMBER_CONFIGURATION
            .Slot0.kD = Constants.Climber.KD;

        CLIMBER_CONFIGURATION
            .MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

}

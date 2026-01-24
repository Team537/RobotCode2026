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
}

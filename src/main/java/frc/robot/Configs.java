package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    
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
            .MotorOutput.Inverted = Constants.Turret.MOTOR_INVERTED;

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


}

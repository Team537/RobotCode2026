package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static class Operator{

        public static class Drive{

            public static final Rotation2d BLUE_ALLIANCE_DRIVER_ROTATION = Rotation2d.kZero;
            public static final Rotation2d RED_ALLIANCE_DRIVER_ROTATION = Rotation2d.kPi;

            public static final double TRANSLATION_INPUT_CURVE_POWER = 2.5;
            public static final double ROTATION_INPUT_CURVE_POWER = 2.5;
            
            public static final double NORMAL_TRANSLATION_MAX_SPEED = 2.0; // Meters per second
            public static final double THROTTLE_TRANSLATION_MAX_SPEED = 4.8; // Meters per second
            public static final double SLOW_TRANSLATION_MAX_SPEED = 0.5; // Meters per second
            public static final double NORMAL_ROTATION_MAX_SPEED = 5.0; // Radians per second
            public static final double THROTTLE_ROTATION_MAX_SPEED = 8.0; // Radians per second
            public static final double SLOW_ROTATION_MAX_SPEED = 1.0; // Radians per second

            public static final double TARGET_TRANSLATION_RADIUS = 2.0;

        }

    }

    public static class Drive {

        // Starting Poses
        public static final Pose2d BLUE_STARTING_POSE = new Pose2d(
            2.0,
            6.0,
            Rotation2d.kZero
        );
        public static final Pose2d RED_STARTING_POSE = new Pose2d(
            2.0,
            6.0,
            Rotation2d.kPi
        );

        // Drive PID Controller Coefficients
        public static final double TRANSLATIONAL_KP = 3.2;
        public static final double TRANSLATIONAL_KI = 0.0;
        public static final double TRANSLATIONAL_KD = 0.05;

        public static final double ROTATIONAL_KP = 5.0;
        public static final double ROTATIONAL_KI = 0.0;
        public static final double ROTATIONAL_KD = 0.1;

        public static final double MAX_TRANSLATIONAL_SPEED = 5.0; // Meters per second
        public static final double MAX_ROTATIONAL_SPEED = 8.0; // Radians per second
        
        // NOTE: This value is only used in the trapezoidal motion profiling of the robot. Other maximums are stored in deploy settings.
        public static final double MAX_TRANSLATIONAL_ACCELERATION = 2.0; // Meters per second squared
        public static final double MAX_ROTATIONAL_ACCELERATION = 2.0;

        public static final File YAGSL_CONFIG = new File(Filesystem.getDeployDirectory(),"swerve/ourSetup");
        public static final double ANGULAR_VELOCITY_COMPENSATION_COEFFICIENT = 0.1;     
        public static final Rotation2d ENCODER_AUTO_SYNCHRONIZE_DEADBAND = Rotation2d.fromDegrees(1.0);

        public static final double TRANSLATIONAL_TOLERANCE = 0.025; // Meters
        public static final Rotation2d ROTATIONAL_TOLERANCE = Rotation2d.fromDegrees(3.5);


    }
    
    public static class VisionOdometryConstants {

        // Center Camera Constants (OLD / EXAMPLE)
        public static final String CENTER_CAMERA_NAME = "back";
        public static final Rotation3d CENTER_CAMERA_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(180));
        public static final Translation3d CENTER_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(-10), 
                Units.inchesToMeters(0), 
                Units.inchesToMeters(0));
        
        public static final Vector<N3> CENTER_SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(4, 4, 8); 
        public static final Vector<N3> CENTER_MULTI_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Intake {
        public static final int INTAKE_LEAD_ID = 1;
        
        public static final int CURRENT_LIMIT = 75;
        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = 0.5;

        public static final double KP = .7;
        public static final double KI = 0;
        public static final double KD = 0.2;

        public static final double GEAR_RATIO = 20;
        public static final double ENCODER_FACTOR = Math.PI * 2 / GEAR_RATIO;

        public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final MotorAlignmentValue MOTOR_ALIGNMENT = MotorAlignmentValue.Aligned;
    }

    public static class IntakePivot {
        public static final int INTAKE_ID = 2;

        public static final int CURRENT_LIMIT = 75;
        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = .5;

        public static final double KP = .7;
        public static final double KI = 0;
        public static final double KD = .2;

        public static final double GEAR_RATIO = 20;
        public static final double ENCODER_FACTOR = Math.PI * 2 / GEAR_RATIO;

        public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Rotation2d INTAKE_START_POS = Rotation2d.fromDegrees(90); //Prevents the intake from going beyond its start positon
        public static final Rotation2d INTAKE_MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d INTAKE_MAX_ANGLE = INTAKE_START_POS; //Prevents the robot from going beyond its maxiumum angle
        public static final Rotation2d INTAKE_RAISED_ANGLE = INTAKE_MIN_ANGLE;
        public static final Rotation2d INTAKE_DEPLOYED_ANGLE = Rotation2d.fromDegrees(30);        

        public static final Rotation2d INTAKE_TOLERANCE_ANGLE = Rotation2d.fromDegrees(5);
    }
}

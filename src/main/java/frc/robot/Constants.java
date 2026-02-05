package frc.robot;

import java.io.File;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.turret.TurretSolver;

public class Constants {

    public static class Operator {

        public static class Drive {

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

    public static class Field {

        public static final double GRAVITY = 9.81; // N/kg

        public static final double AUTONOMOUS_PERIOD = 20.0;
        public static final double TELEOP_PERIOD = 140.0;

        // Shifts

        public static final double AUTO_START = 0.0;
        public static final double AUTO_END   = 20.0;

        // Match period boundaries (elapsed seconds since match start, including autonomous)

        public static final double TRANSITION_START = 20.0;
        public static final double TRANSITION_END   = 30.0;

        public static final double SHIFT_1_START = 30.0;
        public static final double SHIFT_1_END   = 55.0;

        public static final double SHIFT_2_START = 55.0;
        public static final double SHIFT_2_END   = 80.0;

        public static final double SHIFT_3_START = 80.0;
        public static final double SHIFT_3_END   = 105.0;

        public static final double SHIFT_4_START = 105.0;
        public static final double SHIFT_4_END   = 130.0;

        public static final double ENDGAME_START = 130.0;
        public static final double ENDGAME_END    = 160.0;

    }

    public static class Drive {

        // Starting Poses
        public static final Pose2d BLUE_STARTING_POSE = new Pose2d(
            7.013,
            6.085,
            Rotation2d.fromDegrees(-152.5)
        );
        public static final Pose2d RED_STARTING_POSE = new Pose2d(
                2.0,
                6.0,
                Rotation2d.kPi);

        // Drive PID Controller Coefficients
        public static final double TRANSLATIONAL_KP = 3.0;
        public static final double TRANSLATIONAL_KI = 0.0;
        public static final double TRANSLATIONAL_KD = 0.05;

        public static final double ROTATIONAL_KP = 5.0;
        public static final double ROTATIONAL_KI = 0.0;
        public static final double ROTATIONAL_KD = 0.1;

        public static final double MAX_TRANSLATIONAL_SPEED = 5.0; // Meters per second
        public static final double MAX_ROTATIONAL_SPEED = 8.0; // Radians per second

        // NOTE: This value is only used in the trapezoidal motion profiling of the
        // robot. Other maximums are stored in deploy settings.
        public static final double MAX_TRANSLATIONAL_ACCELERATION = 2.0; // Meters per second squared
        public static final double MAX_ROTATIONAL_ACCELERATION = 2.0;

        public static final File YAGSL_CONFIG = new File(Filesystem.getDeployDirectory(), "swerve/ourSetup");
        public static final double ANGULAR_VELOCITY_COMPENSATION_COEFFICIENT = 0.1;
        public static final Rotation2d ENCODER_AUTO_SYNCHRONIZE_DEADBAND = Rotation2d.fromDegrees(1.0);

        public static final double TRANSLATIONAL_TOLERANCE = 0.025; // Meters
        public static final Rotation2d ROTATIONAL_TOLERANCE = Rotation2d.fromDegrees(3.5);

    }

    public static class Turret {

        public static final int TURRET_ID = 20;
        public static final int TURRET_MOTOR_CURRENT_LIMIT = 40;

        // PID
        public static final double KP = 2.5;
        public static final double KI = 0;
        public static final double KD = 0.4;
        public static final double KS = 0.0;
        public static final double KV = 2.36;
        public static final double KA = 0.18;

        public static final boolean MOTOR_INVERTED = false;

        public static final double TURRET_GEAR_REDUCTION = 5.0;
        public static final double TURN_TABLE_RATIO = 24.0 / 200.0;
        public static final double ENCODER_FACTOR = 1.0 / (TURRET_GEAR_REDUCTION * TURN_TABLE_RATIO);

        public static final double OUTPUT_RANGE_MAX = 1;
        public static final double OUTPUT_RANGE_MIN = -1;

        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = 0.5;

        public static final Rotation2d START_POSITION = Rotation2d.kPi;
        public static final Rotation2d MIN_ROTATION = Rotation2d.fromRadians(0.0);
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromRadians(2.0 * Math.PI);

        public static final Rotation2d TURRET_TOLERANCE = Rotation2d.fromRadians(3.0);

        public static final Translation3d TURRET_TRANSLATION = new Translation3d(
            -0.089,
            0.0,
            0.537 //537!!!
        );
        public static final Rotation2d PITCH = Rotation2d.fromDegrees(70.0);
        public static final TurretSolver.Config SOLVER_CONFIG = new TurretSolver.Config(
            Field.GRAVITY,
            0.02,
            15.0, //TODO: replace with actual maximum launch speed once turret constants are added       
            TURRET_TRANSLATION,
            PITCH
        );

    }

    public static final class Shooter {

        public static final int SHOOTER_ID = 21;
        public static final int CURRENT_LIMIT = 75;
        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = .5;

        public static final double KP = .7;
        public static final double KI = 0;
        public static final double KD = .2;
        public static final double KS = 0.0;
        public static final double KV = 2.36;
        public static final double KA = 0.18;

        public static final double GEAR_RATIO = 1.0;
        public static final double WHEEL_RADIUS = 1.985; 
        public static final double ENCOER_FACTOR = 2.0 * Math.PI * WHEEL_RADIUS / GEAR_RATIO;

        public static final double TOLERANCE = 0.1;
        
    }
    public static class Transfer {
        public static final int TRANSFER_MOTOR_ID = 22;

        public static final int CURRENT_LIMIT = 75; //Amps
        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = 0.5;

        public static final double KP = 0.7;
        public static final double KI = 0.0;
        public static final double KD = 0.2;
        public static final double KS = 0.0;
        public static final double KV = 2.36;
        public static final double KA = 0.18;

        public static final double TRANSFER_GEAR_REDUCTION = 1.0;
        public static final double TRANSFER_WHEEL_RADIUS = 1.0;
        public static final double ENCODER_FACTOR = 2.0 * Math.PI * TRANSFER_WHEEL_RADIUS / TRANSFER_GEAR_REDUCTION;

        public static final boolean MOTOR_INVERTED = false;

        public static final double LOAD_SPEED = 1.0;
    }

    public static class VisionOdometryConstants {

        // Center Camera Constants (OLD / EXAMPLE)
        public static final String CENTER_CAMERA_NAME = "back";
        public static final Rotation3d CENTER_CAMERA_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(0));
        public static final Translation3d CENTER_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(10), 
                Units.inchesToMeters(0), 
                Units.inchesToMeters(0));

        public static final Vector<N3> CENTER_SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(4, 4, 8);
        public static final Vector<N3> CENTER_MULTI_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.5, 0.5, 1);
    }

}

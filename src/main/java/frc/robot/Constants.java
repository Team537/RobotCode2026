package frc.robot;

import java.io.File;
import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.field.FieldUtil;
import frc.robot.util.field.regions.CompositeRegion3d;
import frc.robot.util.field.regions.RectangularRegion3d;
import frc.robot.util.field.regions.Region3d;
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

        public static class ErrorSettings {

            public static final double SETTINGS_DELAY_TIME = 0.5;
            public static final double TURRET_OFFSET_INCREASE = 0.1; // Degrees to increase per tick
            public static final int TURRET_OFFSET_DECIMAL_PLACE = 1;
            public static final double SHOOTER_PERCENT_INCREASE = 0.2; // Percent to increase per tick
            public static final int SHOOTER_PERCENT_DECIMAL_PLACE = 1;

        }

    }

    public static class Field {

        public static final double GRAVITY = 9.81; // N/kg

        public static final double AUTONOMOUS_PERIOD = 20.0;
        public static final double TELEOP_PERIOD = 140.0;

        // Shifts

        public static final double AUTO_START = 0.0;
        public static final double AUTO_END = 20.0;

        // Match period boundaries (elapsed seconds since match start, including
        // autonomous)

        public static final double TRANSITION_START = 20.0;
        public static final double TRANSITION_END = 30.0;

        public static final double SHIFT_1_START = 30.0;
        public static final double SHIFT_1_END = 55.0;

        public static final double SHIFT_2_START = 55.0;
        public static final double SHIFT_2_END = 80.0;

        public static final double SHIFT_3_START = 80.0;
        public static final double SHIFT_3_END = 105.0;

        public static final double SHIFT_4_START = 105.0;
        public static final double SHIFT_4_END = 130.0;

        public static final double ENDGAME_START = 130.0;
        public static final double ENDGAME_END = 160.0;

        public static final double FIELD_LENGTH = 16.54;
        public static final double FIELD_WIDTH = 8.07;
        public static final Translation2d FIELD_CENTER = new Translation2d(0.5 * FIELD_LENGTH, 0.5 * FIELD_WIDTH);

        /** Mirror across field center (180° rotation) */
        public static Translation2d flipTranslation(Translation2d t) {
            return new Translation2d(
                    FIELD_LENGTH - t.getX(),
                    FIELD_WIDTH - t.getY());
        }

        /** Mirror Y */
        public static double flipY(double y) {
            return FIELD_WIDTH - y;
        }

        /* --------------------------------------------------------------------- */
        /* Blue / Red Alliance Zone Corners */
        /* --------------------------------------------------------------------- */

        public static final Translation2d BLUE_ALLIANCE_ZONE_C1 = new Translation2d(
                0.000,
                0.000);

        public static final Translation2d BLUE_ALLIANCE_ZONE_C2 = new Translation2d(
                4.028,
                flipY(BLUE_ALLIANCE_ZONE_C1.getY()));

        public static final Translation2d RED_ALLIANCE_ZONE_C1 = flipTranslation(BLUE_ALLIANCE_ZONE_C1);

        public static final Translation2d RED_ALLIANCE_ZONE_C2 = flipTranslation(BLUE_ALLIANCE_ZONE_C2);

        /* --------------------------------------------------------------------- */
        /* Tower Zone Corners (Forbidden in Alliance Regions) */
        /* --------------------------------------------------------------------- */

        public static final Translation2d BLUE_TOWER_C1 = new Translation2d(
                BLUE_ALLIANCE_ZONE_C1.getX(),
                3.253);

        public static final Translation2d BLUE_TOWER_C2 = new Translation2d(
                1.144,
                4.239);

        public static final Translation2d RED_TOWER_C1 = flipTranslation(BLUE_TOWER_C1);

        public static final Translation2d RED_TOWER_C2 = flipTranslation(BLUE_TOWER_C2);

        /* --------------------------------------------------------------------- */
        /* Neutral Zone Corners */
        /* --------------------------------------------------------------------- */

        public static final Translation2d NEUTRAL_ZONE_C1 = new Translation2d(
                5.222,
                BLUE_ALLIANCE_ZONE_C1.getY());

        public static final Translation2d NEUTRAL_ZONE_C2 = flipTranslation(NEUTRAL_ZONE_C1);

        /* --------------------------------------------------------------------- */
        /* Hub Output Zones (Forbidden in Neutral Region) */
        /* --------------------------------------------------------------------- */

        public static final Translation2d BLUE_HUB_OUTPUT_C1 = new Translation2d(
                NEUTRAL_ZONE_C1.getX(),
                3.441);

        public static final Translation2d BLUE_HUB_OUTPUT_C2 = new Translation2d(
                5.600,
                flipY(BLUE_HUB_OUTPUT_C1.getY()));

        public static final Translation2d RED_HUB_OUTPUT_C1 = flipTranslation(BLUE_HUB_OUTPUT_C1);

        public static final Translation2d RED_HUB_OUTPUT_C2 = flipTranslation(BLUE_HUB_OUTPUT_C2);

        /* --------------------------------------------------------------------- */
        /* RectangularRegion3d Definitions (2D → Infinite Z) */
        /* --------------------------------------------------------------------- */

        public static final RectangularRegion3d BLUE_ALLIANCE_ZONE = new RectangularRegion3d(BLUE_ALLIANCE_ZONE_C1,
                BLUE_ALLIANCE_ZONE_C2);

        public static final RectangularRegion3d RED_ALLIANCE_ZONE = new RectangularRegion3d(RED_ALLIANCE_ZONE_C1,
                RED_ALLIANCE_ZONE_C2);

        public static final RectangularRegion3d BLUE_TOWER_ZONE = new RectangularRegion3d(BLUE_TOWER_C1, BLUE_TOWER_C2);

        public static final RectangularRegion3d RED_TOWER_ZONE = new RectangularRegion3d(RED_TOWER_C1, RED_TOWER_C2);

        public static final RectangularRegion3d NEUTRAL_ZONE = new RectangularRegion3d(NEUTRAL_ZONE_C1,
                NEUTRAL_ZONE_C2);

        public static final RectangularRegion3d BLUE_HUB_OUTPUT_ZONE = new RectangularRegion3d(BLUE_HUB_OUTPUT_C1,
                BLUE_HUB_OUTPUT_C2);

        public static final RectangularRegion3d RED_HUB_OUTPUT_ZONE = new RectangularRegion3d(RED_HUB_OUTPUT_C1,
                RED_HUB_OUTPUT_C2);

        /* --------------------------------------------------------------------- */
        /* FieldRegion3d Definitions */
        /* --------------------------------------------------------------------- */

        /** Blue alliance playable region (tower excluded) */
        public static final CompositeRegion3d BLUE_ALLIANCE_REGION = new CompositeRegion3d(
                List.of(BLUE_ALLIANCE_ZONE),
                List.of(BLUE_TOWER_ZONE));

        /** Red alliance playable region (tower excluded) */
        public static final CompositeRegion3d RED_ALLIANCE_REGION = new CompositeRegion3d(
                List.of(RED_ALLIANCE_ZONE),
                List.of(RED_TOWER_ZONE));

        /** Neutral playable region (hub outputs excluded) */
        public static final CompositeRegion3d NEUTRAL_REGION = new CompositeRegion3d(
                List.of(NEUTRAL_ZONE),
                List.of(
                        BLUE_HUB_OUTPUT_ZONE,
                        RED_HUB_OUTPUT_ZONE));

        public static final Translation3d BLUE_HUB_TRANSLATION = new Translation3d(
            4.619,
            4.035,
            1.829
        );

        public static final Translation3d BLUE_DEPO_TRANSLATION = new Translation3d(
            0.390,
            5.956,
            0.0
        );

        public static final Translation3d BLUE_OUTPOST_TRANSLATION = new Translation3d(
            0.390,
            0.626,
            0.0
        );

        public static final CompositeRegion3d BLUE_TRENCH_REGION = new CompositeRegion3d(
            List.of(
                new RectangularRegion3d(
                    new Translation2d(3.996, 8.067),
                    new Translation2d(5.222,6.811)
                ),
                new RectangularRegion3d(
                    new Translation2d(3.996, 1.307),
                    new Translation2d(5.222,0.000)
                )
            )
        );

        public static final Region3d RED_TRENCH_REGION = FieldUtil.flip(BLUE_TRENCH_REGION);

        public static final Region3d TRENCH_REGION = new CompositeRegion3d(List.of(BLUE_TRENCH_REGION,RED_TRENCH_REGION));

    }

    public static class Drive {

        // Starting Poses
        public static final Pose2d BLUE_STARTING_POSE = new Pose2d(
            7.013,
            6.085,
            Rotation2d.kZero
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

        public static final double HOOD_STOW_LOOKAHEAD_TIME = 1.0;

    }

    public static class Turret {

        public static final int TURRET_ID = 56;
        public static final int TURRET_MOTOR_CURRENT_LIMIT = 40;

        public static final int PITCH_SERVO_ID = 1;
       
        public static final int PITCH_CANCODER_ID = 44;

        // PID
        public static final double KP = 3.0;
        public static final double KI = 0.8;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;

        public static final double PITCH_KP = 3.0;
        public static final double PITCH_KI = 0.0;
        public static final double PITCH_KD = 0.0;

        public static final boolean MOTOR_INVERTED = true;

        public static final double TURRET_GEAR_REDUCTION = 5.0;
        public static final double TURN_TABLE_RATIO = 24.0 / 200.0;
        public static final double ENCODER_FACTOR = (TURRET_GEAR_REDUCTION) / (2.0 * Math.PI * TURN_TABLE_RATIO);

        public static final double PITCH_GEAR_RATIO = (26.0 / 447.2);
        public static final double PITCH_ENCODER_FACTOR = PITCH_GEAR_RATIO * (2.0 * Math.PI);

        public static final boolean PITCH_INVERTED = true;

        public static final Rotation2d MAX_PITCH = Rotation2d.fromDegrees(45.0);
        public static final Rotation2d MIN_PITCH = Rotation2d.fromDegrees(3.00);
        public static final Rotation2d HOOD_START_POSITION =Rotation2d.fromDegrees(5.00);
        public static final Rotation2d HOOD_STOW_POSITION = Rotation2d.fromDegrees(3.00);

        public static final double OUTPUT_RANGE_MAX = 1;
        public static final double OUTPUT_RANGE_MIN = -1;

        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = 0.5;

        public static final Rotation2d START_POSITION = Rotation2d.fromRadians(1.5 * Math.PI);
        public static final Rotation2d MIN_ROTATION = Rotation2d.fromRadians(1.0 * Math.PI);
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromRadians(2.0 * Math.PI);

        public static final Rotation2d TURRET_TOLERANCE = Rotation2d.fromRadians(3.0);

        public static final Rotation2d HOOD_TOLERANCE = Rotation2d.fromRadians(3.0);

        public static final Translation3d TURRET_TRANSLATION = new Translation3d(
            -0.089,
            0.0,
            0.537 //537!!!
        );
        public static final TurretSolver.Config SOLVER_CONFIG = new TurretSolver.Config(
            Field.GRAVITY,
            0.02,
            Shooter.MAX_BALL_SPEED, 
            TURRET_TRANSLATION,
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(85),
            5.0,
            MIN_ROTATION,
            MAX_ROTATION
        );

    }

    public static final class Shooter {

        public static final int LEAD_SHOOTER_ID = 53;
        public static final int FOLLOWER_SHOOTER_ID = 52;

        public static final int CURRENT_LIMIT = 75;
        public static final int CURRENT_LOWER_LIMIT = 25; 
        public static final double CURRENT_LOWER_TIME = .5;

        public static final double KP = 1.5;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0.13;
        public static final double KV = 0.25;
        public static final double KA = 0.46;

        public static final double GEAR_RATIO = 20.0 / 24.0;
        public static final double WHEEL_RADIUS = 0.050419; 
        public static final double ENCODER_FACTOR = GEAR_RATIO / (2.0 * Math.PI * WHEEL_RADIUS) ;

        public static final double TOLERANCE = 0.1; // Meters per second

        public static final double[][] WHEEL_SPEED_TO_BALL_SPEED_POINTS = {
            {0.0,0.0},
            {40.0,12.50}
        }; // Meters per second to meters per second

        public static final double MAX_BALL_SPEED = 12.5; // Meters per second

        public static final boolean MOTOR_INVERTED = false;

    }
    public static class Transfer {
        public static final int TRANSFER_MOTOR_ID = 58;

        public static final int CURRENT_LIMIT = 75; //Amps
        public static final int CURRENT_LOWER_LIMIT = 25;
        public static final double CURRENT_LOWER_TIME = 0.5;

        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 2.3;
        public static final double KA = 0.3;

        public static final double TRANSFER_GEAR_REDUCTION = 1.0;
        public static final double TRANSFER_WHEEL_RADIUS = 0.0254;
        public static final double ENCODER_FACTOR = TRANSFER_GEAR_REDUCTION / (2.0 * Math.PI * TRANSFER_WHEEL_RADIUS);

        public static final boolean MOTOR_INVERTED = true;

        public static final double LOAD_SPEED = 16.0;
    }

    public static class VisionOdometryConstants {

        // Center Camera Constants (OLD / EXAMPLE)
        public static final String CENTER_CAMERA_NAME = "back";
        public static final Rotation3d CENTER_CAMERA_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(180));
        public static final Translation3d CENTER_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(-9.725),
                Units.inchesToMeters(0.25),
                Units.inchesToMeters(12.75));

        public static final Vector<N3> CENTER_SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(4, 4, 8);
        public static final Vector<N3> CENTER_MULTI_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.5, 0.5, 1);
        
        // Left Camera Constants
        public static final String LEFT_CAMERA_NAME = "left";
        public static final Rotation3d LEFT_CAMERA_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(90));
        public static final Translation3d LEFT_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(-2.250),
                Units.inchesToMeters(9.725),
                Units.inchesToMeters(10.750));

        public static final Vector<N3> LEFT_SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(4, 4, 8);
        public static final Vector<N3> LEFT_MULTI_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.5, 0.5, 1);

        // Right Camera Constants (OLD / EXAMPLE)
        public static final String RIGHT_CAMERA_NAME = "right";
        public static final Rotation3d RIGHT_CAMERA_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(-90));
        public static final Translation3d RIGHT_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(-2.250),
                Units.inchesToMeters(-9.725),
                Units.inchesToMeters(10.750));

        public static final Vector<N3> RIGHT_SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(4, 4, 8);
        public static final Vector<N3> RIGHT_MULTI_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class RaycastConstants {

        // IPs
        public static final String JETSON_IP = "10.5.37.42";

        // Ports 
        // NOTE: THESE MUST ALL BE DIFFERENT AND BE BETWEEN 5800 and 5810!!!!
        public static final int UDP_SENDER_PORT = 5804;
        public static final int IMU_RESET_PORT = 5805;
        public static final int ROBOT_DETECTION_PORT = 5806;
        public static final int TIME_SYNC_PORT = 5807;


        // Visualization.
        public static final String RAYCAST_FIELD_NAME = "raycast-field";
    }

    public static class IntakeRoller {

        public static final int INTAKE_ID = 54;
        
        public static final int CURRENT_LIMIT = 75;
        public static final int CURRENT_LOWER_LIMIT = 40;
        public static final double CURRENT_LOWER_TIME = 0.5;


        public static final double GEAR_RATIO = 4.0;
        public static final double ENCODER_FACTOR = GEAR_RATIO / (2.0 * Math.PI);

        public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final double INTAKE_POWER = 1.0; //percent of max speed
    }

    public static class IntakePivot {

        public static final int INTAKE_ID = 55;
        public static final int CANCODER_ID = 43;

        public static final int CURRENT_LIMIT = 75;
        public static final int CURRENT_LOWER_LIMIT = 40;
        public static final double CURRENT_LOWER_TIME = .5;

        public static final double KP = 3.0; // Volts per radian
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;

        public static final double GEAR_RATIO = 54.0;
        public static final double ROTOR_TO_SENSOR_RATIO = GEAR_RATIO;
        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0 / (2.0 * Math.PI);

        public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Rotation2d INTAKE_START_POS = Rotation2d.fromDegrees(125.0); //Prevents the intake from going beyond its start positon
        public static final Rotation2d INTAKE_MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d INTAKE_MAX_ANGLE = INTAKE_START_POS; //Prevents the robot from going beyond its maxiumum angle
        public static final Rotation2d INTAKE_RAISED_ANGLE = Rotation2d.fromDegrees(110.0);
        public static final Rotation2d INTAKE_DEPLOYED_ANGLE = INTAKE_MIN_ANGLE;        

        public static final Rotation2d INTAKE_TOLERANCE_ANGLE = Rotation2d.fromDegrees(3);
        
    }

}

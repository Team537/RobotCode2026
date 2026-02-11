package frc.robot;

import java.io.File;
import org.apache.commons.text.similarity.LevenshteinDistance;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.field.regions.CompositeRegion3d;
import frc.robot.util.field.regions.RectangularRegion3d;

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

    public static class Field {

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
            Rotation2d.kPi
        );

        // Drive PID Controller Coefficients
        public static final double TRANSLATIONAL_KP = 3.0;
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

        public static final class Field {

            public static final double FIELD_LENGTH = 16.54;
            public static final double FIELD_WIDTH = 8.07;

            /** Mirror across field center (180° rotation) */
            private static Translation2d flipTranslation(Translation2d t) {
                return new Translation2d(
                    FIELD_LENGTH - t.getX(),
                    FIELD_WIDTH  - t.getY()
                );
            }

            /** Mirror Y */
            private static double flipY(double y) {
                return FIELD_WIDTH - y;
            }

            /* --------------------------------------------------------------------- */
        /*  Blue / Red Alliance Zone Corners                                      */
        /* --------------------------------------------------------------------- */

        public static final Translation2d BLUE_ALLIANCE_ZONE_C1 = new Translation2d(
            0.000,
            0.000
        );

        public static final Translation2d BLUE_ALLIANCE_ZONE_C2 = new Translation2d(
            4.028,
            flipY(BLUE_ALLIANCE_ZONE_C1.getY())
        );

        public static final Translation2d RED_ALLIANCE_ZONE_C1 =
            flipTranslation(BLUE_ALLIANCE_ZONE_C1);

        public static final Translation2d RED_ALLIANCE_ZONE_C2 =
            flipTranslation(BLUE_ALLIANCE_ZONE_C2);

        /* --------------------------------------------------------------------- */
        /*  Tower Zone Corners (Forbidden in Alliance Regions)                    */
        /* --------------------------------------------------------------------- */

        public static final Translation2d BLUE_TOWER_C1 = new Translation2d(
            BLUE_ALLIANCE_ZONE_C1.getX(),
            3.253
        );

        public static final Translation2d BLUE_TOWER_C2 = new Translation2d(
            1.144,
            4.239
        );

        public static final Translation2d RED_TOWER_C1 =
            flipTranslation(BLUE_TOWER_C1);

        public static final Translation2d RED_TOWER_C2 =
            flipTranslation(BLUE_TOWER_C2);

        /* --------------------------------------------------------------------- */
        /*  Neutral Zone Corners                                                  */
        /* --------------------------------------------------------------------- */

        public static final Translation2d NEUTRAL_ZONE_C1 = new Translation2d(
            5.222,
            BLUE_ALLIANCE_ZONE_C1.getY()
        );

        public static final Translation2d NEUTRAL_ZONE_C2 =
            flipTranslation(NEUTRAL_ZONE_C1);

        /* --------------------------------------------------------------------- */
        /*  Hub Output Zones (Forbidden in Neutral Region)                        */
        /* --------------------------------------------------------------------- */

        public static final Translation2d BLUE_HUB_OUTPUT_C1 = new Translation2d(
            NEUTRAL_ZONE_C1.getX(),
            3.441
        );

        public static final Translation2d BLUE_HUB_OUTPUT_C2 = new Translation2d(
            5.600,
            flipY(BLUE_HUB_OUTPUT_C1.getY())
        );

        public static final Translation2d RED_HUB_OUTPUT_C1 =
            flipTranslation(BLUE_HUB_OUTPUT_C1);

        public static final Translation2d RED_HUB_OUTPUT_C2 =
            flipTranslation(BLUE_HUB_OUTPUT_C2);

        /* --------------------------------------------------------------------- */
        /*  RectangularRegion3d Definitions (2D → Infinite Z)                    */
        /* --------------------------------------------------------------------- */

        public static final RectangularRegion3d BLUE_ALLIANCE_ZONE =
            new RectangularRegion3d(BLUE_ALLIANCE_ZONE_C1, BLUE_ALLIANCE_ZONE_C2);

        public static final RectangularRegion3d RED_ALLIANCE_ZONE =
            new RectangularRegion3d(RED_ALLIANCE_ZONE_C1, RED_ALLIANCE_ZONE_C2);

        public static final RectangularRegion3d BLUE_TOWER_ZONE =
            new RectangularRegion3d(BLUE_TOWER_C1, BLUE_TOWER_C2);

        public static final RectangularRegion3d RED_TOWER_ZONE =
            new RectangularRegion3d(RED_TOWER_C1, RED_TOWER_C2);

        public static final RectangularRegion3d NEUTRAL_ZONE =
            new RectangularRegion3d(NEUTRAL_ZONE_C1, NEUTRAL_ZONE_C2);

        public static final RectangularRegion3d BLUE_HUB_OUTPUT_ZONE =
            new RectangularRegion3d(BLUE_HUB_OUTPUT_C1, BLUE_HUB_OUTPUT_C2);

        public static final RectangularRegion3d RED_HUB_OUTPUT_ZONE =
            new RectangularRegion3d(RED_HUB_OUTPUT_C1, RED_HUB_OUTPUT_C2);

        /* --------------------------------------------------------------------- */
        /*  FieldRegion3d Definitions                                             */
        /* --------------------------------------------------------------------- */

        /** Blue alliance playable region (tower excluded) */
        public static final CompositeRegion3d BLUE_ALLIANCE_REGION =
            new CompositeRegion3d(
                List.of(BLUE_ALLIANCE_ZONE),
                List.of(BLUE_TOWER_ZONE)
            );

        /** Red alliance playable region (tower excluded) */
        public static final CompositeRegion3d RED_ALLIANCE_REGION =
            new CompositeRegion3d(
                List.of(RED_ALLIANCE_ZONE),
                List.of(RED_TOWER_ZONE)
            );

        /** Neutral playable region (hub outputs excluded) */
        public static final CompositeRegion3d NEUTRAL_REGION =
            new CompositeRegion3d(
                List.of(NEUTRAL_ZONE),
                List.of(
                    BLUE_HUB_OUTPUT_ZONE,
                    RED_HUB_OUTPUT_ZONE
                )
            );


        }


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

    public static class RaycastConstants {

        // IPs
        public static final String JETSON_IP = "10.5.37.42";

        // Ports 
        // NOTE: THESE MUST ALL BE DUIFFERENT AND BE BETWEEN 5800 and 5810!!!!
        public static final int TCP_PORT = 5805;
        public static final int ROBOT_DETECTION_PORT = 5806;
        public static final int TIME_SYNC_PORT = 5807;


        // Visualization.
        public static final String RAYCAST_FIELD_NAME = "raycast-field";
    }
}

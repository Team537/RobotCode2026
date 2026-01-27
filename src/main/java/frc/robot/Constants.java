package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        public static final double ROTATIONAL_KP = 10.0;
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

        public static final double BALL_HUNT_REPLANNING_DISTANCE = 0.2; // How far the the target must travel before the path is recalculated, in meters
        public static final double BALL_HUNT_DIRECT_DRIVE_DISTANCE = 1.0; // How far away the target must be before the robot desides its better to pathfind, in meters
        public static final double BALL_HUNT_TARGET_VELOCITY = 4.0; // The velocity the robot should travel to pick up the balls in m/s
        public static final double BALL_HUNT_ROTATION_FORCE_POWER = 10.0; // The power to scale the translation component of velocity when forcing rotation for ball hunt
        public static final double PATROL_REGION_TIME = 2.0; // The amount of time the robot must spend in the region before it starts spinning, in seconds
        public static final double PATROL_ANGULAR_VELOCITY = 1.0; // The angular speed that the robot should spin while patrolling

        // Ball Hunt Weights
        private static final double DEFAULT_TRANSLATION_DISTANCE_WEIGHT = 0.5;
        private static final double DEFAULT_ROTATION_DISTANCE_WEIGHT = 1.0;
        private static final double DEFAULT_TRANSLATIONAL_VELOCITY_WEIGHT = 0.5;
        private static final double DEFAULT_ROTATIONAL_VELOCITY_WEIGHT = 0.5;
        private static final double DEFAULT_CONFIDENCE_WEIGHT = 1.0;


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
                6.000,
                flipY(BLUE_HUB_OUTPUT_C1.getY())
            );

            public static final Translation2d RED_HUB_OUTPUT_C1 =
                flipTranslation(BLUE_HUB_OUTPUT_C1);

            public static final Translation2d RED_HUB_OUTPUT_C2 =
                flipTranslation(BLUE_HUB_OUTPUT_C2);

            /* --------------------------------------------------------------------- */
            /*  FieldArea3d Definitions (2D → Infinite Z)                             */
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

            private static final double CENTER_X = FIELD_LENGTH / 2.0;
            private static final double CENTER_BUFFER = 0.5;
            private static final double HALF_BUFFER = CENTER_BUFFER / 2.0;

            private static final RectangularRegion3d CENTER_FORBIDDEN_STRIP =
                new RectangularRegion3d(
                    new Translation2d(CENTER_X - HALF_BUFFER, 0.0),
                    new Translation2d(CENTER_X + HALF_BUFFER, FIELD_WIDTH)
                );

            /** Red half of the field */
            private static final RectangularRegion3d RED_SIDE_REGION =
                new RectangularRegion3d(
                    new Translation2d(CENTER_X, 0.0),
                    new Translation2d(FIELD_LENGTH, FIELD_WIDTH)
                );

            /** Blue half of the field */
            private static final RectangularRegion3d BLUE_SIDE_REGION =
                new RectangularRegion3d(
                    new Translation2d(0.0, 0.0),
                    new Translation2d(CENTER_X, FIELD_WIDTH)
                );

            /** Neutral region usable by blue (auto-safe) */
            public static final CompositeRegion3d BLUE_NEUTRAL_REGION =
                new CompositeRegion3d(
                    List.of(NEUTRAL_REGION),
                    List.of(
                        RED_SIDE_REGION,
                        CENTER_FORBIDDEN_STRIP
                    )
                );

            /** Neutral region usable by red (auto-safe) */
            public static final CompositeRegion3d RED_NEUTRAL_REGION =
                new CompositeRegion3d(
                    List.of(NEUTRAL_REGION),
                    List.of(
                        BLUE_SIDE_REGION,
                        CENTER_FORBIDDEN_STRIP
                    )
                );




            public static final Translation2d fieldCenter = new Translation2d(FIELD_LENGTH, FIELD_WIDTH).div(2.0);

            public static final RectangularRegion3d TEST_REGION_NEUTRAL = 
                new RectangularRegion3d(
                    fieldCenter.minus(new Translation2d(2.0,3.0)),
                    fieldCenter.plus(new Translation2d(2.0,3.0))
                );

            public static final RectangularRegion3d TEST_REGION_BLUE = 
                new RectangularRegion3d(
                    fieldCenter.minus(new Translation2d(1.0,3.0)).minus(new Translation2d(6.0,0.0)),             
                    fieldCenter.plus(new Translation2d(1.0,3.0)).minus(new Translation2d(6.0,0.0))
                );

            public static final RectangularRegion3d TEST_REGION_RED = 
                new RectangularRegion3d(
                    fieldCenter.minus(new Translation2d(1.0,3.0)).plus(new Translation2d(6.0,0.0)),             
                    fieldCenter.plus(new Translation2d(1.0,3.0)).plus(new Translation2d(6.0,0.0))
                );

            public static final CompositeRegion3d TEST_REGION_COMPOSITE =
                new CompositeRegion3d(List.of(TEST_REGION_NEUTRAL,TEST_REGION_BLUE,TEST_REGION_RED));

        }
    }

    public static class Intake {

        public static final Transform2d intakeTransform = new Transform2d(
            new Translation2d(
                    0.15,
                    0.0),
            Rotation2d.kZero);

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
}

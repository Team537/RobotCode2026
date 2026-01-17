package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.field.FieldArea3d;
import frc.robot.util.field.FieldRegion3d;

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
        public static final double TRANSLATIONAL_KP = 3.0;
        public static final double TRANSLATIONAL_KI = 0.0;
        public static final double TRANSLATIONAL_KD = 0.1;

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

        public static final double TRANSLATIONAL_TOLERANCE = 0.1;
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
        /*  FieldArea3d Definitions (2D → Infinite Z)                             */
        /* --------------------------------------------------------------------- */

        public static final FieldArea3d BLUE_ALLIANCE_ZONE =
            new FieldArea3d(BLUE_ALLIANCE_ZONE_C1, BLUE_ALLIANCE_ZONE_C2);

        public static final FieldArea3d RED_ALLIANCE_ZONE =
            new FieldArea3d(RED_ALLIANCE_ZONE_C1, RED_ALLIANCE_ZONE_C2);

        public static final FieldArea3d BLUE_TOWER_ZONE =
            new FieldArea3d(BLUE_TOWER_C1, BLUE_TOWER_C2);

        public static final FieldArea3d RED_TOWER_ZONE =
            new FieldArea3d(RED_TOWER_C1, RED_TOWER_C2);

        public static final FieldArea3d NEUTRAL_ZONE =
            new FieldArea3d(NEUTRAL_ZONE_C1, NEUTRAL_ZONE_C2);

        public static final FieldArea3d BLUE_HUB_OUTPUT_ZONE =
            new FieldArea3d(BLUE_HUB_OUTPUT_C1, BLUE_HUB_OUTPUT_C2);

        public static final FieldArea3d RED_HUB_OUTPUT_ZONE =
            new FieldArea3d(RED_HUB_OUTPUT_C1, RED_HUB_OUTPUT_C2);

        /* --------------------------------------------------------------------- */
        /*  FieldRegion3d Definitions                                             */
        /* --------------------------------------------------------------------- */

        /** Blue alliance playable region (tower excluded) */
        public static final FieldRegion3d BLUE_ALLIANCE_REGION =
            new FieldRegion3d(
                List.of(BLUE_ALLIANCE_ZONE),
                List.of(BLUE_TOWER_ZONE)
            );

        /** Red alliance playable region (tower excluded) */
        public static final FieldRegion3d RED_ALLIANCE_REGION =
            new FieldRegion3d(
                List.of(RED_ALLIANCE_ZONE),
                List.of(RED_TOWER_ZONE)
            );

        /** Neutral playable region (hub outputs excluded) */
        public static final FieldRegion3d NEUTRAL_REGION =
            new FieldRegion3d(
                List.of(NEUTRAL_ZONE),
                List.of(
                    BLUE_HUB_OUTPUT_ZONE,
                    RED_HUB_OUTPUT_ZONE
                )
            );


        }


    }
    
}

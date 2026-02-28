package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.field.regions.Region3d;

public class SwerveUtil {
    
    private static final double LOOK_AHEAD_STEP = 0.1;
    
    /**
     * Predicts whether the robot will enter the given region within a future time window.
     *
     * <p>This method forward-integrates the robot pose assuming constant chassis
     * speeds and checks if any projected position lies inside the region.</p>
     *
     * <p>Note:
     * <ul>
     *   <li>Assumes constant velocity (no acceleration model).</li>
     *   <li>Only checks the robot's translation (ignores rotation footprint).</li>
     *   <li>Resolution depends on LOOK_AHEAD_STEP.</li>
     * </ul>
     * </p>
     *
     * @param robotPose         Current field-relative pose of the robot.
     * @param robotVelocity     Current chassis speeds (robot-relative).
     * @param region            Region to test for intersection.
     * @param maxLookAheadTime  Time horizon to simulate (seconds).
     * @return true if the projected path enters the region.
     */
    public static boolean willRobotEnterRegion(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Region3d region,
            double maxLookAheadTime) {

        // Step forward in time from now to the lookahead limit.
        for (double t = 0.0; t <= maxLookAheadTime; t += LOOK_AHEAD_STEP) {

            // Convert constant chassis speeds into a twist over Δt.
            Twist2d twist = ChassisSpeeds.fromFieldRelativeSpeeds(robotVelocity,robotPose.getRotation()).toTwist2d(t);

            // Integrate pose forward using exponential coordinates.
            Pose2d projectedPose = robotPose.exp(twist);

            // Check if the projected translation lies inside the region.
            if (region.contains(projectedPose.getTranslation())) {
                return true;
            }
        }

        return false;
    }
        
}

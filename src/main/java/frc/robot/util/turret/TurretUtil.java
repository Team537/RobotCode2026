package frc.robot.util.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

public class TurretUtil {

    private static final double TWO_PI = 2.0 * Math.PI;

    /**
     * Resolves a desired angle to the closest valid equivalent angle.
     * <p>
     * Angles are treated as absolute (not normalized). The desired angle may be
     * shifted by multiples of 2π to find the closest equivalent angle to the
     * current angle that lies within the allowed range.
     * <p>
     * If no equivalent angle exists within the range, the result is clamped to
     * the nearest bound.
     *
     * @param current Current angle (absolute)
     * @param desired Desired angle (absolute)
     * @param min     Minimum allowed angle (absolute)
     * @param max     Maximum allowed angle (absolute)
     * @return Closest valid angle to command
     */
    public static Rotation2d resolveClosestValidAngle(
            Rotation2d current,
            Rotation2d desired,
            Rotation2d min,
            Rotation2d max) {
        double cur = current.getRadians();
        double des = desired.getRadians();
        double minA = min.getRadians();
        double maxA = max.getRadians();

        // Find the wrap that puts desired closest to current
        double kCenter = Math.round((cur - des) / TWO_PI);

        double bestAngle = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;

        // Check nearby wraps to handle boundary cases
        for (long k = (long) kCenter - 1; k <= (long) kCenter + 1; k++) {
            double candidate = des + k * TWO_PI;

            if (candidate < minA || candidate > maxA) {
                continue;
            }

            double error = Math.abs(candidate - cur);
            if (error < bestError) {
                bestError = error;
                bestAngle = candidate;
            }
        }

        // 2. FALLBACK: If no wrapped version fits in the range, 
        // find which boundary is closest to the target angle itself.
        if (Double.isNaN(bestAngle)) {
            // Normalize the distance from target to min/max boundaries
            // This ensures 350 is seen as only 10 degrees away from 0
            double distToMin = Math.abs(MathUtil.angleModulus(des - minA));
            double distToMax = Math.abs(MathUtil.angleModulus(des - maxA));

            bestAngle = (distToMin < distToMax) ? minA : maxA;
        }

        return new Rotation2d(MathUtil.clamp(bestAngle,minA,maxA));
    }

    private static final double BALL_SPEED_GAIN = 4.88072;
    private static final double YAW_COS_COEFFICIENT = -1.20843;
    private static final double PITCH_SIN_COEFFICIENT = 11.20532;
    private static final double PITCH_PHASE_OFFSET_RAD = Math.toRadians(12.65252);
    private static final double MODEL_BIAS = -26.67181;

    /**
     * Computes the wheel surface speed required to produce a given ball exit speed
     * using the regression model derived from shooter characterization data.
     *
     * @param ballSpeed Ball exit velocity in meters per second.
     * @param yaw Turret yaw angle.
     * @param pitch Hood pitch angle.
     * @return Wheel surface speed in meters per second.
     */
    public static double wheelSurfaceSpeedFromBallSpeed(
            double ballSpeed,
            Rotation2d yaw,
            Rotation2d pitch) {

        return (BALL_SPEED_GAIN * ballSpeed)
            + (YAW_COS_COEFFICIENT * Math.cos(yaw.getRadians()))
            + (PITCH_SIN_COEFFICIENT * Math.sin(pitch.getRadians() + PITCH_PHASE_OFFSET_RAD))
            + MODEL_BIAS;
    }

    /**
     * Computes the resulting ball exit speed produced by a given wheel surface speed.
     *
     * @param wheelSurfaceSpeed Flywheel surface speed in meters per second.
     * @param yaw Turret yaw angle.
     * @param pitch Hood pitch angle.
     * @return Ball exit velocity in meters per second.
     */
    public static double ballSpeedFromWheelSurfaceSpeed(
            double wheelSurfaceSpeed,
            Rotation2d yaw,
            Rotation2d pitch) {

        return (
            wheelSurfaceSpeed
            - (YAW_COS_COEFFICIENT * Math.cos(yaw.getRadians()))
            - (PITCH_SIN_COEFFICIENT * Math.sin(pitch.getRadians() + PITCH_PHASE_OFFSET_RAD))
            - MODEL_BIAS
        ) / BALL_SPEED_GAIN;
    }


}

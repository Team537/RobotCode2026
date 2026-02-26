package frc.robot.util.turret;

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

        // If no valid wrapped angle exists, clamp to nearest bound
        if (Double.isNaN(bestAngle)) {
            if (cur < minA) {
                bestAngle = minA;
            } else if (cur > maxA) {
                bestAngle = maxA;
            } else {
                double toMin = Math.abs(minA - cur);
                double toMax = Math.abs(maxA - cur);
                bestAngle = (toMin < toMax) ? minA : maxA;
            }
        }

        return new Rotation2d(bestAngle);
    }

    private static InterpolatingDoubleTreeMap wheelToBall = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap ballToWheel = new InterpolatingDoubleTreeMap();

    static {
        for (double[] point : Constants.Shooter.WHEEL_SPEED_TO_BALL_SPEED_POINTS) {
            double wheel = point[0];
            double ball = point[1];

            wheelToBall.put(wheel, ball);
            ballToWheel.put(ball, wheel);
        }
    }

    /**
     * Gets the wheel speed that would shoot the ball at provided speed from a regression model.
     * @param ballSpeed The speed of the ball in meters per second.
     * @return The wheel surface speed that will shoot the ball in meters per second.
     */
    public static double wheelSurfaceSpeedFromBallSpeed(double ballSpeed) {
        return ballToWheel.get(ballSpeed);
    }

    /**
     * Gets the ball speed that a given wheel speed would shoot it out from a regression model.
     * @param wheelSurfaceSpeed The speed of the surface of the flywheel in meters per second.
     * @return The speed the ball will shoot out in meters per second.
     */
    public static double ballSpeedFromWheelSurfaceSpeed(double wheelSurfaceSpeed) {
        return wheelToBall.get(wheelSurfaceSpeed);
    }


}

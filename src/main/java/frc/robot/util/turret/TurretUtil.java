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

        return new Rotation2d(MathUtil.clamp(bestAngle, minA, maxA));
    }

    private static final double SQUARE_COSINE_COEFFICIENT = -1.127;
    private static final double COSINE_COEFFICIENT = -0.843;
    private static final double STATIC_COEFFICIENT = 2.161;

    public static Rotation2d pitchOffsetFromYaw(Rotation2d yaw) {

        return Rotation2d.fromDegrees(
            SQUARE_COSINE_COEFFICIENT * yaw.getCos() * yaw.getCos() +
            COSINE_COEFFICIENT * yaw.getCos() +
            STATIC_COEFFICIENT
        );

    }

    public static Rotation2d getVelocityCompensatedAngle(Rotation2d angle, double rotationalVelocity) {
        return angle.minus(Rotation2d.fromRadians(rotationalVelocity * Constants.Turret.TURRET_LOOKAHEAD_TIME));
    }

}

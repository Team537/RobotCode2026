package frc.robot.util.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class TurretSolver {

    private static final double PITCH_STEP = 0.02;
    private static final double TIME_EPS = 1e-4;
    private static final int MAX_ITER = 20;

    private static final double PENALTY_MAX_SPEED = 1.0;
    private static final double PENALTY_MAX_HEIGHT = 1.0;
    private static final double PENALTY_INVALID_YAW = 25.0;
    private static final double PENALTY_WEIGHT = 100.0;

    private TurretSolver() {}

    // ============================ OUTPUT STATE ============================

    public static class State {
        private final double launchVelocity;
        private final Rotation2d yaw;
        private final Rotation2d pitch;
        private final double maxHeight;
        private final double impactVelocity;
        private final boolean valid;
        private final double time;

        public State(double v, Rotation2d yaw, Rotation2d pitch, boolean valid, double h, double impactV, double time) {
            this.launchVelocity = v;
            this.yaw = yaw;
            this.pitch = pitch;
            this.valid = valid;
            this.maxHeight = h;
            this.impactVelocity = impactV;
            this.time = time;
        }

        public double getLaunchVelocity() { return launchVelocity; }
        public Rotation2d getYaw() { return yaw; }
        public Rotation2d getPitch() { return pitch; }
        public boolean isValid() { return valid; }
        public double getMaxHeight() { return maxHeight; }
        public double getImpactVelocity() { return impactVelocity; }
        public double getTime() {return time; }
    }

    // ============================ CONFIG ============================

    public static class Config {
        public final double gravity;
        public final double poseLatency;
        public final double maxLaunchSpeed;
        public final Translation3d turretOffset;

        public final Rotation2d minPitch;
        public final Rotation2d maxPitch;
        public final double maxHeight;

        // NEW
        public final Rotation2d minYaw;
        public final Rotation2d maxYaw;

        // === Original Constructor (assume full yaw range) ===
        public Config(
                double gravity,
                double poseLatency,
                double maxLaunchSpeed,
                Translation3d turretOffset,
                Rotation2d minPitch,
                Rotation2d maxPitch,
                double maxHeight
        ) {
            this(
                    gravity,
                    poseLatency,
                    maxLaunchSpeed,
                    turretOffset,
                    minPitch,
                    maxPitch,
                    maxHeight,
                    Rotation2d.fromDegrees(-180),
                    Rotation2d.fromDegrees(180)
            );
        }

        // === New Constructor With Yaw Limits ===
        public Config(
                double gravity,
                double poseLatency,
                double maxLaunchSpeed,
                Translation3d turretOffset,
                Rotation2d minPitch,
                Rotation2d maxPitch,
                double maxHeight,
                Rotation2d minYaw,
                Rotation2d maxYaw
        ) {
            this.gravity = gravity;
            this.poseLatency = poseLatency;
            this.maxLaunchSpeed = maxLaunchSpeed;
            this.turretOffset = turretOffset;
            this.minPitch = minPitch;
            this.maxPitch = maxPitch;
            this.maxHeight = maxHeight;

            this.minYaw = minYaw;
            this.maxYaw = maxYaw;
        }
    }

    // ============================ SOLVE ============================

    public static State solve(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation3d targetTranslation,
            Config config
    ) {

        Pose2d correctedPose =
                robotPose.exp(robotVelocity.toTwist2d(config.poseLatency));

        Translation3d robotTranslation =
                new Translation3d(correctedPose.getX(), correctedPose.getY(), 0.0);

        Translation3d rotatedOffset =
                config.turretOffset.rotateBy(
                        new Rotation3d(0, 0, correctedPose.getRotation().getRadians()));

        Translation3d muzzle = robotTranslation.plus(rotatedOffset);

        Translation3d d = targetTranslation.minus(muzzle);

        double rx = d.getX();
        double ry = d.getY();
        double rz = d.getZ();

        double vrx = robotVelocity.vxMetersPerSecond;
        double vry = robotVelocity.vyMetersPerSecond;

        double bestValidCost = Double.POSITIVE_INFINITY;
        double bestApproxCost = Double.POSITIVE_INFINITY;

        double bestV = 0.0;
        Rotation2d bestYaw = new Rotation2d();
        Rotation2d bestPitch = new Rotation2d();
        double bestH = 0.0;
        double bestImpactV = 0.0;

        double approxV = 0.0;
        Rotation2d approxYaw = new Rotation2d();
        Rotation2d approxPitch = new Rotation2d();
        double approxH = 0.0;
        double approxImpactV = 0.0;

        boolean foundValid = false;

        for (double phi = config.minPitch.getRadians(); phi <= config.maxPitch.getRadians(); phi += PITCH_STEP) {

            double t = solveTimeOfFlight(rx, ry, rz, vrx, vry, phi, config.gravity);
            if (t <= 0) continue;

            double v = (rz + 0.5 * config.gravity * t * t) / (t * Math.sin(phi));
            if (v <= 0) continue;

            double apexHeight =
                config.turretOffset.getZ()
                + (v * v * Math.sin(phi) * Math.sin(phi))
                / (2.0 * config.gravity);

            double thetaField = Math.atan2((ry / t) - vry, (rx / t) - vrx);
            Rotation2d yaw = Rotation2d.fromRadians(thetaField).minus(correctedPose.getRotation());

            double impactV = v * Math.sin(phi) - config.gravity * t;

            // Hard physical constraint
            if (impactV >= 0) continue;

            // Cost function for selecting among multiple valid ballistic solutions: 
            // - We square the launch speed (v^2) so that higher launch velocities are 
            // penalized more aggressively than with a linear term. This biases the 
            // solver toward lower-speed shots when several trajectories can reach 
            // the target. 
            // - We divide by |impactV| so that, for the same launch speed, trajectories 
            // with higher (faster) downward impact velocity are preferred, since they 
            // tend to be less sensitive to small disturbances at the target. double cost = (v * v) / Math.abs(impactV);
            double cost = (v * v) / Math.abs(impactV);

            // Penalty system for approximate solutions
            double penalty = 0.0;

            if (v > config.maxLaunchSpeed) {
                penalty += Math.pow(v - config.maxLaunchSpeed, 2) * PENALTY_MAX_SPEED;
            }

            if (apexHeight > config.maxHeight) {
                penalty += Math.pow(apexHeight - config.maxHeight, 2) * PENALTY_MAX_HEIGHT;
            }

            if (!isYawInRange(yaw, config.minYaw, config.maxYaw)) {
                double yawError =
                    Math.max(
                        config.minYaw.minus(yaw).getRadians(),
                        yaw.minus(config.maxYaw).getRadians()
                    );
                penalty += yawError * yawError * PENALTY_INVALID_YAW;
            }

            boolean valid =
                penalty == 0
                && v <= config.maxLaunchSpeed
                && apexHeight <= config.maxHeight
                && isYawInRange(yaw, config.minYaw, config.maxYaw);

            if (valid) {
                if (cost < bestValidCost) {
                    bestValidCost = cost;
                    bestV = v;
                    bestYaw = yaw;
                    bestPitch = Rotation2d.fromRadians(phi);
                    bestH = apexHeight;
                    bestImpactV = impactV;
                    foundValid = true;
                }
            }

            double approxCost = cost + penalty * PENALTY_WEIGHT;

            if (approxCost < bestApproxCost) {
                bestApproxCost = approxCost;
                approxV = v;
                approxYaw = yaw;
                approxPitch = Rotation2d.fromRadians(phi);
                approxH = apexHeight;
                approxImpactV = impactV;
            }
        }

        if (foundValid) {
            return new State(bestV, bestYaw, bestPitch, true, bestH, bestImpactV, 0);
        }

        return new State(approxV, approxYaw, approxPitch, false, approxH, approxImpactV, 0);
    }

    // ============================ TIME SOLVER ============================

    private static double solveTimeOfFlight(
            double rx, double ry, double rz,
            double vrx, double vry,
            double phi, double g
    ) {
        double horizontalDist = Math.hypot(rx, ry);
        double t = horizontalDist / 10.0; // better initial guess
        if (t < 0.05) t = 0.05;

        for (int i = 0; i < MAX_ITER; i++) {
            double f = calculateF(t, rx, ry, rz, vrx, vry, phi, g);
            double fPrime = derivativeF(t, rx, ry, rz, vrx, vry, phi, g);
            if (Math.abs(fPrime) < 1e-6) break;

            double tNext = t - f / fPrime;
            if (tNext <= 0) tNext = t * 0.5;

            if (Math.abs(tNext - t) < TIME_EPS) return tNext;
            t = tNext;
        }

        return -1;
    }

    private static double calculateF(
            double t, double rx, double ry, double rz,
            double vrx, double vry,
            double phi, double g
    ) {
        double dx = (rx / t) - vrx;
        double dy = (ry / t) - vry;

        double horizontalRequired = Math.hypot(dx, dy);
        double horizontalAvailable = (rz + 0.5 * g * t * t) / (t * Math.tan(phi));

        return horizontalRequired - horizontalAvailable;
    }

    private static double derivativeF(
            double t, double rx, double ry, double rz,
            double vrx, double vry,
            double phi, double g
    ) {
        double dx = (rx / t) - vrx;
        double dy = (ry / t) - vry;
        double dDx = -rx / (t * t);
        double dDy = -ry / (t * t);

        double horizontalRequired = Math.hypot(dx, dy);
        double dHorizontalRequired;
        if (horizontalRequired < 1e-9) {
            // When the required horizontal velocity is effectively zero,
            // treat its derivative as zero to avoid division by zero and
            // keep the Newton iteration numerically stable.
            dHorizontalRequired = 0.0;
        } else {
            dHorizontalRequired = (dx * dDx + dy * dDy) / horizontalRequired;
        }

        double numerator = rz + 0.5 * g * t * t;
        double denominator = t * Math.tan(phi);

        double dHorizontalAvailable = ((g * t) * denominator - numerator * Math.tan(phi)) / (denominator * denominator);

        return dHorizontalRequired - dHorizontalAvailable;
    }

    private static boolean isYawInRange(Rotation2d yaw,
                                    Rotation2d min,
                                    Rotation2d max) {

        double y = MathUtil.angleModulus(yaw.getRadians());
        double minR = MathUtil.angleModulus(min.getRadians());
        double maxR = MathUtil.angleModulus(max.getRadians());

        if (minR <= maxR) {
            return y >= minR && y <= maxR;
        } else {
            // Wrapped range (ex: 270° → 90°)
            return y >= minR || y <= maxR;
        }
    }

}

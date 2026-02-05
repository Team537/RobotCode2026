package frc.robot.util.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * TurretSolver provides ballistic calculations for determining
 * turret yaw and required launch velocity to hit a target.
 *
 * <p>This class is intended to be a pure math utility with no
 * internal state. All physical and robot-specific parameters
 * are supplied via {@link Config}.</p>
 */
public final class TurretSolver {

    private static final double DT = 0.0001;
    private static final double EPSILON = 0.01;
    private static final int MAX_ITER = 10;

    private TurretSolver() {
        // Utility class; prevent instantiation
    }

    /**
     * Represents the output of the turret ballistic solver.
     */
    public static class State {

        /** Required projectile launch velocity (m/s). */
        private final double launchVelocity;

        /** Required turret yaw angle (field-relative). */
        private final Rotation2d launchAngle;

        /** Whether a valid physical trajectory exists. */
        private final boolean validTrajectory;

        /**
         * Creates a solver State.
         *
         * @param launchVelocity required projectile exit velocity (m/s)
         * @param launchAngle required turret yaw angle
         * @param validTrajectory whether the State is physically achievable
         */
        public State(double launchVelocity, Rotation2d launchAngle, boolean validTrajectory) {
            this.launchVelocity = launchVelocity;
            this.launchAngle = launchAngle;
            this.validTrajectory = validTrajectory;
        }

        /** @return required projectile exit velocity (m/s) */
        public double getLaunchVelocity() {
            return launchVelocity;
        }

        /** @return required turret yaw angle */
        public Rotation2d getLaunchAngle() {
            return launchAngle;
        }

        /** @return true if the solver found a valid trajectory */
        public boolean getValidTrajectory() {
            return validTrajectory;
        }
    }

    /**
     * Immutable configuration parameters for the turret ballistic solver.
     *
     * <p>This class encapsulates all physical constants, robot geometry,
     * and fixed parameters required by the solver. It should be constructed
     * externally and passed into the solver on each call.</p>
     */
    public static class Config {

        /** Gravitational acceleration (m/s^2). Positive value. */
        private final double gravity;

        /** Pose estimation latency to compensate for (seconds). */
        private final double poseLatency;

        /** Maximum launch speed (m/s) */
        private final double maxLaunchSpeed;

        /**
         * Offset from the robot reference frame to the projectile
         * exit point (meters).
         */
        private final Translation3d turretOffset;

        /**
         * Fixed turret pitch angle relative to horizontal.
         * This is assumed constant for all shots.
         */
        private final Rotation2d turretPitch;

        /**
         * Creates a turret solver configuration.
         *
         * @param gravity gravitational acceleration (m/s^2)
         * @param poseLatency pose estimation latency to compensate for (s)
         * @param maxLaunchSpeed maximum launch speed (m/s), set to Double.POSITIVE_INFINITY for unbounded
         * @param turretOffset translation from robot origin to muzzle (m)
         * @param turretPitch fixed turret pitch angle
         */
        public Config(
            double gravity,
            double poseLatency,
            double maxLaunchSpeed,
            Translation3d turretOffset,
            Rotation2d turretPitch
        ) {
            this.gravity = gravity;
            this.poseLatency = poseLatency;
            this.maxLaunchSpeed = maxLaunchSpeed;
            this.turretOffset = turretOffset;
            this.turretPitch = turretPitch;
        }

        /** @return gravitational acceleration (m/s^2) */
        public double getGravity() {
            return gravity;
        }

        /** @return pose estimation latency (seconds) */
        public double getPoseLatency() {
            return poseLatency;
        }

        /** @return maxmimum launch speed (m/s) */
        public double getMaxLaunchSpeed() {
            return maxLaunchSpeed;
        }

        /** @return turret muzzle offset from robot origin (meters) */
        public Translation3d getTurretOffset() {
            return turretOffset;
        }

        /** @return fixed turret pitch angle */
        public Rotation2d getTurretPitch() {
            return turretPitch;
        }
    }

    /**
     * Computes the required turret yaw and projectile launch velocity
     * needed to hit a stationary target, accounting for robot motion
     * and gravity.
     *
     * <p>This method performs a purely ballistic calculation. It does
     * not command hardware, cache state, or apply safety logic.
     * All physical constants and robot geometry must be provided via {@link Config}.</p>
     *
     * <h3>Coordinate Frames</h3>
     * <ul>
     *   <li>{@code robotPose} is field-relative.</li>
     *   <li>{@code robotVelocity} is field-relative ({@link ChassisSpeeds}).</li>
     *   <li>{@code targetTranslation} is field-relative.</li>
     *   <li>Returned turret yaw is robot-relative.</li>
     * </ul>
     *
     * <h3>Units</h3>
     * <ul>
     *   <li>Distances: meters</li>
     *   <li>Angles: radians</li>
     *   <li>Velocities: meters per second</li>
     * </ul>
     *
     * @param robotPose current estimated robot pose (field-relative)
     * @param robotVelocity current field chassis speeds (field-relative)
     * @param targetTranslation field-relative target position
     * @param config immutable solver configuration parameters
     * @return a {@link State} containing turret yaw, launch velocity, and validity
     */
    public static State solve(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation3d targetTranslation,
            Config config
    ) {

        // Apply latency compensation to get predicted robot pose at launch
        Pose2d correctedPose = robotPose.exp(robotVelocity.toTwist2d(config.getPoseLatency()));

        // Turret base position in field frame (using corrected robot pose)
        Translation3d robotTranslation =
                new Translation3d(correctedPose.getX(), correctedPose.getY(), 0.0);

        // Rotate turret offset by robot yaw to get field-relative muzzle position
        Translation3d rotatedTurretOffset =
                config.getTurretOffset()
                    .rotateBy(new Rotation3d(0.0, 0.0, correctedPose.getRotation().getRadians()));

        Translation3d turretTranslation = robotTranslation.plus(rotatedTurretOffset);

        // Vector from muzzle to target
        Translation3d targetDisplacement = targetTranslation.minus(turretTranslation);
        double rx = targetDisplacement.getX();
        double ry = targetDisplacement.getY();
        double rz = targetDisplacement.getZ();

        double vrx = robotVelocity.vxMetersPerSecond;
        double vry = robotVelocity.vyMetersPerSecond;

        double phi = config.getTurretPitch().getRadians();
        double g = config.getGravity();

        // Solve for projectile time-of-flight using Newton-Raphson
        double t = solveTimeOfFlight(rx, ry, rz, vrx, vry, phi, g);

        // If no valid solution, return invalid State immediately
        if (t <= 0.0) {
            return new State(0.0, Rotation2d.fromRadians(0.0), false);
        }

        // Compute required muzzle speed
        double v = (rz + 0.5 * g * t * t) / (t * Math.sin(phi));

        // Compute field-relative yaw to hit target
        double fieldTheta = Math.atan2((ry / t) - vry, (rx / t) - vrx);

        // Convert to robot-relative yaw
        Rotation2d theta = Rotation2d.fromRadians(fieldTheta).minus(correctedPose.getRotation());

        // Check solution feasibility
        boolean valid =
                (0.0 < v) && (v <= config.maxLaunchSpeed) &&
                (v * Math.sin(phi) - g * t < 0.0); // descending on arrival

        return new State(v, theta, valid);
    }

    // -------------------------- Numerical Root Solver --------------------------

    private static double solveTimeOfFlight(double rx, double ry, double rz,
                                            double vrx, double vry,
                                            double phi, double g) {

        double t = 1.0; // initial guess (seconds)

        for (int i = 0; i < MAX_ITER; i++) {
            double f = calculateF(t, rx, ry, rz, vrx, vry, phi, g);

            // Central difference for numerical derivative
            double fPrime =
                    (calculateF(t + DT, rx, ry, rz, vrx, vry, phi, g) -
                    calculateF(t - DT, rx, ry, rz, vrx, vry, phi, g)) / (2 * DT);

            // Avoid divide-by-zero
            if (Math.abs(fPrime) < 1e-9) break;

            double tNext = t - f / fPrime;

            // Convergence check
            if (Math.abs(tNext - t) < EPSILON) {
                return tNext;
            }

            t = tNext;
        }

        // Failed to Converge
        return -1.0;

    }

    private static double calculateF(double t, double rx, double ry, double rz,
                                    double vrx, double vry,
                                    double phi, double g) {

        double dx = (rx / t) - vrx;
        double dy = (ry / t) - vry;

        double lhs = Math.hypot(dx, dy); // horizontal speed required
        double rhs = (rz + 0.5 * g * t * t) / (t * Math.tan(phi)); // horizontal speed available

        return lhs - rhs;
    }
}
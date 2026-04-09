package frc.robot.util.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class TurretSolver {

    private static final int MAX_ITER = 20;

    private TurretSolver() {
    }

    // ============================ OUTPUT STATE ============================

    public static class State {
        private final double launchVelocity;
        private final Rotation2d yaw;
        private final Rotation2d pitch;
        private final double time;

        public State(double v, Rotation2d yaw, Rotation2d pitch, double time) {
            this.launchVelocity = v;
            this.yaw = yaw;
            this.pitch = pitch;
            this.time = time;
        }

        public double getLaunchVelocity() {
            return launchVelocity;
        }

        public Rotation2d getYaw() {
            return yaw;
        }

        public Rotation2d getPitch() {
            return pitch;
        }

        public double getTime() {
            return time;
        }
    }

    // ============================ CONFIG ============================

    public static class Config {

        public final double poseLatency;
        public final Translation3d turretOffset;

        public final InterpolatingDoubleTreeMap hoodAngleMap;
        public final InterpolatingDoubleTreeMap shooterVelocityMap;
        public final InterpolatingDoubleTreeMap timeMap;

        public Config(
                double poseLatency,
                Translation3d turretOffset,
                InterpolatingDoubleTreeMap hoodAngleMap,
                InterpolatingDoubleTreeMap shooterVelocityMap,
                InterpolatingDoubleTreeMap timeMap) {
            this.poseLatency = poseLatency;
            this.turretOffset = turretOffset;
            this.hoodAngleMap = hoodAngleMap;
            this.shooterVelocityMap = shooterVelocityMap;
            this.timeMap = timeMap;
        }
    }

    // ============================ SOLVE ============================

    public static State solve(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation3d targetTranslation,
            Config config) {

        Pose2d correctedPose = robotPose.exp(robotVelocity.toTwist2d(config.poseLatency));

        double lastTime = 0.0;
        Rotation2d lastHoodAngle = Rotation2d.fromDegrees(45.0);
        Rotation2d lastTurretYaw = Rotation2d.fromDegrees(0.0);
        double lastShooterVelocity = 0.0;

        for (int i = 0; i < MAX_ITER; i++) {

            Translation2d linearOffset = new Translation2d(
                    robotVelocity.vxMetersPerSecond * lastTime,
                    robotVelocity.vyMetersPerSecond * lastTime);
            Pose2d estimatedPose = new Pose2d(
                    correctedPose.getTranslation().plus(linearOffset),
                    correctedPose.getRotation() // keep original rotation, do not integrate angular velocity
            );

            Translation2d rotatedOffset = config.turretOffset.toTranslation2d().rotateBy(estimatedPose.getRotation());

            Translation2d muzzle = estimatedPose.getTranslation().plus(rotatedOffset);

            double horizontalDistance = targetTranslation.toTranslation2d().minus(muzzle).getNorm();

            lastTime = config.timeMap.get(horizontalDistance);
            lastHoodAngle = Rotation2d.fromDegrees(config.hoodAngleMap.get(horizontalDistance));
            lastTurretYaw = TurretUtil.getVelocityCompensatedAngle(
                    targetTranslation.toTranslation2d().minus(muzzle).getAngle().minus(estimatedPose.getRotation()),
                    robotVelocity.omegaRadiansPerSecond);
            lastShooterVelocity = config.shooterVelocityMap.get(horizontalDistance);

        }

        return new State(lastShooterVelocity, lastTurretYaw, lastHoodAngle, lastTime);

    }

}

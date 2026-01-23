package frc.robot.util.field;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.field.regions.Region3d;

/**
 * Simple simulation utility for spawning and removing fuel objects.
 *
 * - Fuels spawn periodically
 * - Total count is capped
 * - Fuels are removed when the robot gets close
 *
 * Intended to be used as a Supplier<List<Fuel>>.
 */
public class FuelFieldSimulation {

    private final List<Fuel> fuels = new ArrayList<>();
    private final Random rng = new Random();

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final Region3d spawnRegion;

    private final int maxFuelCount;
    private final double spawnPeriod;
    private final double pickupDistance;
    private final double minForwardSpeed;

    private final Transform2d intakeTransform;

    private double lastSpawnTime = 0.0;

    private static final Field2d field = new Field2d();

    public FuelFieldSimulation(
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier,
            Transform2d intakeTransform,
            Region3d spawnRegion,
            int maxFuelCount,
            double spawnPeriod,
            double pickupDistance,
            double minForwardSpeed
    ) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.intakeTransform = intakeTransform;
        this.spawnRegion = spawnRegion;
        this.maxFuelCount = maxFuelCount;
        this.spawnPeriod = spawnPeriod;
        this.pickupDistance = pickupDistance;
        this.minForwardSpeed = minForwardSpeed;
    }

    /**
     * Call this periodically (e.g. every robot loop).
     */
    public void update() {
        double now = Timer.getFPGATimestamp();

        // Remove collected fuels based on intake alignment and forward motion
        Pose2d robotPose = robotPoseSupplier.get();
        Transform2d intakeTransform = this.intakeTransform; // your intake offset
        Translation2d intakePos = robotPose.transformBy(intakeTransform).getTranslation();

        // Robot velocity (field-relative)
        ChassisSpeeds robotVel = robotVelocitySupplier.get(); // meters/sec in field frame
        Translation2d robotTransVel = new Translation2d(
            robotVel.vxMetersPerSecond,
            robotVel.vyMetersPerSecond
        );

        Iterator<Fuel> it = fuels.iterator();
        while (it.hasNext()) {
            Fuel fuel = it.next();
            Translation2d fuelPos = fuel.getTranslation().toTranslation2d();

            // Vector from intake to fuel
            Translation2d toFuel = fuelPos.minus(intakePos);

            // 1️ Check distance from intake to fuel
            double dist = toFuel.getNorm();
            if (dist > pickupDistance) {
                continue;
            }

            // 2 Check that intake is moving generally forward toward the fuel
            // Project velocity onto the vector toward fuel
            double forwardComponent = robotTransVel.dot(toFuel.div(dist));

            // Only collect if moving mostly toward the target
            if (forwardComponent < minForwardSpeed) {
                continue;
            }

            // Passed all checks: remove this fuel
            it.remove();
        }

        // Spawn new fuel if allowed
        if (fuels.size() < maxFuelCount && now - lastSpawnTime >= spawnPeriod) {
            spawnFuel();
            lastSpawnTime = now;
        }

        SmartDashboard.putNumber("Number of Fuel",getFuels().size());
        visualizeFuels();
    }

    /**
     * Returns the current list of fuels.
     * Safe to pass directly into commands.
     */
    public List<Fuel> getFuels() {
        Pose2d robotPose = robotPoseSupplier.get();

        fuels.sort((a, b) -> {
            double da = a.getTranslation()
                    .toTranslation2d()
                    .getDistance(robotPose.getTranslation());
            double db = b.getTranslation()
                    .toTranslation2d()
                    .getDistance(robotPose.getTranslation());
            return Double.compare(da, db);
        });

        return fuels;
    }

    private void spawnFuel() {
        // Simple rejection sampling inside region bounds
        Translation3d min = spawnRegion.getBounds().getMinimumCorner();
        Translation3d max = spawnRegion.getBounds().getMaximumCorner();

        for (int i = 0; i < 20; i++) {
            double x = lerp(min.getX(), max.getX(), rng.nextDouble());
            double y = lerp(min.getY(), max.getY(), rng.nextDouble());
            Translation3d candidate = new Translation3d(x, y, 0.0);

            if (spawnRegion.contains(candidate)) {
                fuels.add(new Fuel(candidate));
                return;
            }
        }
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private void visualizeFuels() {
        field.getObject("Fuel")
            .setPoses(
                fuels.stream()
                    .map(fuel -> new Pose2d(fuel.getTranslation().toTranslation2d(), Rotation2d.kZero))
                    .toArray(Pose2d[]::new)
            );
        field.getObject("Robot")
            .setPose(robotPoseSupplier.get());
        SmartDashboard.putData("Fuel Field", field);
    }   
}
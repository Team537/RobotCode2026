package frc.robot.util.field;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    private final Region3d spawnRegion;

    private final int maxFuelCount;
    private final double spawnPeriodSeconds;
    private final double pickupDistanceMeters;

    private double lastSpawnTime = 0.0;

    private static final Field2d field = new Field2d();

    public FuelFieldSimulation(
            Supplier<Pose2d> robotPoseSupplier,
            Region3d spawnRegion,
            int maxFuelCount,
            double spawnPeriodSeconds,
            double pickupDistanceMeters
    ) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.spawnRegion = spawnRegion;
        this.maxFuelCount = maxFuelCount;
        this.spawnPeriodSeconds = spawnPeriodSeconds;
        this.pickupDistanceMeters = pickupDistanceMeters;
    }

    /**
     * Call this periodically (e.g. every robot loop).
     */
    public void update() {
        double now = Timer.getFPGATimestamp();

        // Remove collected fuels
        Pose2d robotPose = robotPoseSupplier.get();
        Iterator<Fuel> it = fuels.iterator();
        while (it.hasNext()) {
            Fuel fuel = it.next();
            double dist = fuel.getTranslation()
                    .toTranslation2d()
                    .getDistance(robotPose.getTranslation());

            if (dist < pickupDistanceMeters) {
                it.remove();
            }
        }

        // Spawn new fuel if allowed
        if (fuels.size() < maxFuelCount && now - lastSpawnTime >= spawnPeriodSeconds) {
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
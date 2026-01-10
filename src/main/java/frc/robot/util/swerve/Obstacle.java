package frc.robot.util.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a circular 2D obstacle for pathfinding purposes.
 */
public class Obstacle {

    private final Translation2d center;
    private final double radius;

    /**
     * Creates a new Obstacle.
     *
     * @param center The center of the obstacle in meters.
     * @param radius The radius of the obstacle in meters.
     */
    public Obstacle(Translation2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    /**
     * Returns the center of the obstacle.
     *
     * @return Center as a {@link Translation2d}.
     */
    public Translation2d getCenter() {
        return center;
    }

    /**
     * Returns the radius of the obstacle.
     *
     * @return Radius in meters.
     */
    public double getRadius() {
        return radius;
    }

    /**
     * Checks if a point is touching (or inside) this obstacle.
     *
     * @param point The point to test.
     * @return True if the point is within the obstacle's radius.
     */
    public boolean isTouching(Translation2d point) {
        return center.getDistance(point) <= radius;
    }

    /**
     * Checks if another obstacle is touching or overlapping this obstacle.
     *
     * @param other The other obstacle to test.
     * @return True if the two obstacles are overlapping or touching.
     */
    public boolean isTouching(Obstacle other) {
        return center.getDistance(other.center) <= (radius + other.radius);
    }

    /**
     * Calculates the distance from a point to the edge of this obstacle.
     *
     * @param point The point to measure from.
     * @return Distance in meters (negative if inside the obstacle).
     */
    public double distanceTo(Translation2d point) {
        return center.getDistance(point) - radius;
    }

    /**
     * Calculates the distance to another obstacle (edge-to-edge).
     *
     * @param other The other obstacle.
     * @return Distance in meters (negative if overlapping).
     */
    public double distanceTo(Obstacle other) {
        return center.getDistance(other.center) - (radius + other.radius);
    }

    /**
     * Converts this circular obstacle into a bounding box representation
     * suitable for PathPlannerDynamicObstacle (as diagonal corners).
     *
     * @return Pair of bottom-left and top-right corners.
     */
    public Pair<Translation2d, Translation2d> toPathPlannerObstacle() {
        Translation2d bottomLeft = center.minus(new Translation2d(radius, radius));
        Translation2d topRight   = center.plus(new Translation2d(radius, radius));
        return Pair.of(bottomLeft, topRight);
    }

    @Override
    public String toString() {
        return String.format("Obstacle(center=%s, radius=%.2f)", center, radius);
    }
}
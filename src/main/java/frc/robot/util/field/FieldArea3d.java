package frc.robot.util.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Axis-aligned rectangular prism in field coordinates.
 * 
 * Can be constructed from either:
 *  - Two 3D corners (finite volume)
 *  - Two 2D corners (infinitely tall in Z)
 *
 * Corner order does NOT matter.
 */
public class FieldArea3d {

    // Minimum corner (xMin, yMin, zMin)
    private final Translation3d minCorner;

    // Maximum corner (xMax, yMax, zMax)
    private final Translation3d maxCorner;

    /**
     * Construct a finite 3D field area from two opposite corners.
     */
    public FieldArea3d(Translation3d a, Translation3d b) {
        // Normalize corners so minCorner truly holds minimum values
        this.minCorner = new Translation3d(
            Math.min(a.getX(), b.getX()),
            Math.min(a.getY(), b.getY()),
            Math.min(a.getZ(), b.getZ())
        );

        this.maxCorner = new Translation3d(
            Math.max(a.getX(), b.getX()),
            Math.max(a.getY(), b.getY()),
            Math.max(a.getZ(), b.getZ())
        );
    }

    /**
     * Construct a 2D field area that extends infinitely in the Z direction.
     * Useful for field zones that should apply regardless of robot height.
     */
    public FieldArea3d(Translation2d a, Translation2d b) {
        // X/Y are bounded, Z is unbounded
        this.minCorner = new Translation3d(
            Math.min(a.getX(), b.getX()),
            Math.min(a.getY(), b.getY()),
            Double.NEGATIVE_INFINITY
        );

        this.maxCorner = new Translation3d(
            Math.max(a.getX(), b.getX()),
            Math.max(a.getY(), b.getY()),
            Double.POSITIVE_INFINITY
        );
    }

    /**
     * Check whether a 3D point lies inside this field area.
     * Boundaries are inclusive.
     */
    public boolean contains(Translation3d point) {
        return point.getX() >= minCorner.getX() &&
               point.getX() <= maxCorner.getX() &&
               point.getY() >= minCorner.getY() &&
               point.getY() <= maxCorner.getY() &&
               point.getZ() >= minCorner.getZ() &&
               point.getZ() <= maxCorner.getZ();
    }
}

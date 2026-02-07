package frc.robot.util.field.regions;

import java.util.List;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents a composite 3D field region composed of:
 *  - Allowed areas (where the point may be)
 *  - Forbidden areas (which override allowed areas)
 *
 * A point is considered inside the region if:
 *  - It is NOT inside any forbidden area, and
 *  - It IS inside at least one allowed area
 *
 * This is useful for defining zones with exclusions, such as:
 *  - "Anywhere in the alliance zone but not below the tower"
 *  - "Anywhere in the neutral zone but not below the hub"
 */
public class CompositeRegion3d implements Region3d {

    // Areas where the point is allowed to exist
    private final List<Region3d> allowed;

    // Areas that explicitly exclude the point, even if allowed elsewhere
    private final List<Region3d> forbidden;

    /**
     * Construct a region with allowed areas only.
     * No forbidden zones are applied.
     */
    public CompositeRegion3d(List<Region3d> allowed) {
        this.allowed = allowed;
        this.forbidden = List.of(); // Immutable empty list
    }

    /**
     * Construct a region with both allowed and forbidden areas.
     * Forbidden areas take precedence over allowed areas.
     */
    public CompositeRegion3d(List<Region3d> allowed, List<Region3d> forbidden) {
        this.allowed = allowed;
        this.forbidden = forbidden;
    }

    /**
     * Check whether a 3D point lies within this region.
     *
     * @param point Field-relative point to test
     * @return true if the point is inside at least one allowed area
     *         and not inside any forbidden area
     */
    @Override
    public boolean contains(Translation3d point) {
        // First, reject points inside any forbidden area
        for (Region3d area : forbidden) {
            if (area.contains(point)) {
                return false;
            }
        }

        // Then, accept points inside at least one allowed area
        for (Region3d area : allowed) {
            if (area.contains(point)) {
                return true;
            }
        }

        // Not forbidden, but also not explicitly allowed
        return false;
    }

    /**
     * Gets the area's rectangular bounding box
     */
    @Override
    public RectangularRegion3d getBounds() {
        if (allowed.isEmpty()) {
            return null;
        }

        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;

        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;

        for (Region3d region : allowed) {
            RectangularRegion3d bounds = region.getBounds();
            if (bounds == null) {
                continue;
            }

            Translation3d min = bounds.getMinimumCorner();
            Translation3d max = bounds.getMaximumCorner();

            minX = Math.min(minX, min.getX());
            minY = Math.min(minY, min.getY());
            minZ = Math.min(minZ, min.getZ());

            maxX = Math.max(maxX, max.getX());
            maxY = Math.max(maxY, max.getY());
            maxZ = Math.max(maxZ, max.getZ());
        }

        // If all bounds were null
        if (minX == Double.POSITIVE_INFINITY) {
            return null;
        }

        return new RectangularRegion3d(
            new Translation3d(minX, minY, minZ),
            new Translation3d(maxX, maxY, maxZ)
        );
    }
}


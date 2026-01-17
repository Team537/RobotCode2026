package frc.robot.util.field;

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
 *  - "Anywhere in the neutral zone but net below the hub"
 */
public class FieldRegion3d {

    // Areas where the point is allowed to exist
    private final List<FieldArea3d> allowed;

    // Areas that explicitly exclude the point, even if allowed elsewhere
    private final List<FieldArea3d> forbidden;

    /**
     * Construct a region with allowed areas only.
     * No forbidden zones are applied.
     */
    public FieldRegion3d(List<FieldArea3d> allowed) {
        this.allowed = allowed;
        this.forbidden = List.of(); // Immutable empty list
    }

    /**
     * Construct a region with both allowed and forbidden areas.
     * Forbidden areas take precedence over allowed areas.
     */
    public FieldRegion3d(List<FieldArea3d> allowed, List<FieldArea3d> forbidden) {
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
    public boolean contains(Translation3d point) {
        // First, reject points inside any forbidden area
        for (FieldArea3d area : forbidden) {
            if (area.contains(point)) {
                return false;
            }
        }

        // Then, accept points inside at least one allowed area
        for (FieldArea3d area : allowed) {
            if (area.contains(point)) {
                return true;
            }
        }

        // Not forbidden, but also not explicitly allowed
        return false;
    }
}


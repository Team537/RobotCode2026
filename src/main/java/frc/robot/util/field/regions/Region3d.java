package frc.robot.util.field.regions;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public interface Region3d {

    /** Returns true if the given 3D point is inside this region. */
    boolean contains(Translation3d point);

    /**
     * Returns true if the given 2D point is inside this region.
     * Automatically promotes the 2D point to 3D with z = 0.
     */
    default boolean contains(Translation2d point) {
        return contains(new Translation3d(point.getX(), point.getY(), 0.0));
    }

    /** Returns the bounding rectangular region of this 3D region. */
    RectangularRegion3d getBounds();
}

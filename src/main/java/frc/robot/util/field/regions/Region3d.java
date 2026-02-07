package frc.robot.util.field.regions;

import edu.wpi.first.math.geometry.Translation3d;

public interface Region3d {
    
    boolean contains(Translation3d point);

    RectangularRegion3d getBounds();

}

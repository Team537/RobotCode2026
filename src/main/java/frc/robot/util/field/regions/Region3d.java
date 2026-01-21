package frc.robot.util.field.regions;

import edu.wpi.first.math.geometry.Translation3d;

public interface Region3d {
    
    public boolean contains(Translation3d point);

    public RectangularRegion3d getBounds();

}

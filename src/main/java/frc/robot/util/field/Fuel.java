package frc.robot.util.field;

import edu.wpi.first.math.geometry.Translation3d;

public class Fuel {
    
    private final Translation3d translation;

    public Fuel(Translation3d translation) {
        this.translation = translation;
    }

    public Translation3d getTranslation() {
        return translation; 
    }

}

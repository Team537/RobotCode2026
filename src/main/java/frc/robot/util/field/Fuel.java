package frc.robot.util.field;

import edu.wpi.first.math.geometry.Translation3d;

public class Fuel {
    
    private final Translation3d translation;
    private final double confidence;

    public Fuel(Translation3d translation,double confidence) {
        this.translation = translation;
        this.confidence = confidence;
    }

    public Translation3d getTranslation() {
        return translation; 
    }

    public double getConfidence() {
        return confidence;
    }

}

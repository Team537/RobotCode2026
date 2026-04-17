package frc.robot.util.turret;

import edu.wpi.first.math.geometry.Translation3d;

public record TargetingData(Translation3d translation, TargetingStrategy strategy) {
    
}

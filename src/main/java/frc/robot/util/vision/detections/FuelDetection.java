package frc.robot.util.vision.detections;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A generic dataclass used to represent fuel detection data sent from the NVIDIA Jetson Orion Nano Super.
 */
public class FuelDetection {
    
    public final int id;
    public final double x, y, z;
    public final double aiScore, distanceScore, rotationScore;

    /**
     * Creates a new FuelDetection object with the following properties:
     * 
     * @param id The tracking ID of this fuel instance.
     * @param x The fuel's X position.
     * @param y The fuel's Y position.
     * @param z The fuel's Z position.
     * @param aiScore The fuel's AI confidence score (0-1)
     * @param distanceScore PLACEHOLDER
     * @param rotationScore PLACEHOLDER
     */
    public FuelDetection(Integer id, double x, double y, double z, double aiScore, double distanceScore, double rotationScore) {
       this.id = id;
        
        this.x = x;
        this.y = y;
        this.z = z;

        this.aiScore = aiScore;
        this.distanceScore = distanceScore;
        this.rotationScore = rotationScore;
    }


    /*
     * -----------------------------------------------------
     * GETTERS
     * -----------------------------------------------------
     */

    /**
     * Returns this {@code FuelDetection}'s position as a translation 2D (x, y)
     * 
     * @return This {@code FuelDetection}'s position as a translation 2D (x, y)
     */
    public Translation2d getPoseAsTranslation2d() {
        return new Translation2d(this.x, this.y);
    }

    /**
     * Returns this {@code FuelDetection}'s position as a translation 3D (x, y, z)
     * 
     * @return This {@code FuelDetection}'s position as a translation 3D (x, y, z)
     */
    public Translation3d getPoseTranslation3d() {
        return new Translation3d(this.x, this.y, this.z);
    }

    /**
     * Returns the mathematical sum of all three detection scores.
     * 
     * @return the mathematical sum of all three detection scores.
     */
    public double getScoreSum() {
        return this.aiScore + this.distanceScore + this.rotationScore;
    }

    /**
     * Returns the mathematical product of all three detection scores. This is more robust against poor quality in just one of the detection scores.
     * 
     * @return The mathematical product of all three detection scores.
     */
    public double getScoreProduct() {
        return this.aiScore * this.distanceScore * this.rotationScore;
    }
}

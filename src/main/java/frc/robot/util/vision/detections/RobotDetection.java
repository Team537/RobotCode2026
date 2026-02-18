package frc.robot.util.vision.detections;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A generic dataclass used to represent robot detection data sent from the NVIDIA Jetson Orion Nano Super.
 * 
 * <b> AVOID USING PUBLIC ATTRIBUTES!! THEY MAY BE REMOVED LATER </b>
 */
public class RobotDetection {

    // Properties
    public final double x, y, z;
    public final int teamNumber;
    public final double teamNumberConfidence;
    public final Alliance allianceColor;

    /** <b> PLACEHOLDER (Still need to add this in the code ) Use `getRobotRadius` instead.  </b>*/
    public final double radius = 0.0;

    /**
     * Creates a new {@code} RobotDetection} object from given information.
     * 
     * @param x This robot's X coordinate. (fwd/bck)
     * @param y This robot's Y coordinate (right/left)
     * @param z This robot's Z coordinate (height)
     * @param teamNumber This robot's team number.
     * @param allianceColor The alliance this robot belongs to.
     */
    public RobotDetection(double x, double y, double z, int teamNumber, float teamNumberConfidence, Alliance allianceColor) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.teamNumber = teamNumber;
        this.teamNumberConfidence = teamNumberConfidence;
        this.allianceColor = allianceColor;
    }


    /*
     * -----------------------------------------------------
     * GETTERS
     * -----------------------------------------------------
     */

    /**
     * Returns this {@code RobotDetection}'s position as a translation 2D (x, y)
     * 
     * @return This {@code RobotDetection}'s position as a translation 2D (x, y)
     */
    public Translation2d getPoseAsTranslation2d() {
        return new Translation2d(this.x, this.y);
    }

    /**
     * Returns this {@code RobotDetection}'s position as a translation 3D (x, y, z)
     * 
     * @return This {@code RobotDetection}'s position as a translation 3D (x, y, z)
     */
    public Translation3d getPoseTranslation3d() {
        return new Translation3d(this.x, this.y, this.z);
    }

    /**
     * Returns the placeholder radius representing a robot.
     * 
     * @return 0.0
     */
    public double getRobotRadius() {
        return this.radius;
    }

    /*
     * -----------------------------------------------------
     * OVERRIDES
     * -----------------------------------------------------
     */

    /**
     * Overridden `toString()` method to aid in data visualization. This if formatted as follows:
     * str = "[%.3f : %.3ff]: Position: (%.3f, %.3f, %.3f)" in the order: teamNumber, allianceColor, x, y, z
     *
     * @return A string representation of the `RobotDetection` object.
     */
    @Override
    public String toString() {
        String STR_BASE = "[%s : %s]: Position: (%.3f, %.3f, %.3f) ";
        return String.format(STR_BASE, this.teamNumber, this.allianceColor, this.x, this.y, this.z);
    }
}

package frc.robot.util.network;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A single UDP message payload that will be serialized to JSON by Gson.
 *
 * Shape examples:
 *  - {"robot_pose":{"translation":{"x":..,"y":..,"z":..},"yaw_rad":..},"zero_imu":false}
 */
public final class UDPPacket {

    // Snake case is used here to match the coprocessor's naming conventions.
    // The name of these fields directly play into the JSON data structure.
    public RobotPose robot_pose;   // optional
    public Capture capture;        // optional

    // -------------------------
    // Nested message types
    // -------------------------
    public static final class RobotPose {
        public Translation translation; // required if robot_pose is present
        public double yaw_rad;

        private RobotPose() {}
    }

    public static final class Translation {
        public double x;
        public double y;
        public double z;

        private Translation() {}
    }

    public static final class Capture {
        public boolean saveInputFrame, saveOutputFrame, saveDepthFrame;

        private Capture() {}
    }

    // -------------------------
    // Builder
    // -------------------------
    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private Double x, y, z;
        private Double yawRad;
        private Boolean saveInputFrame, saveOutputFrame, saveDepthFrame;
        private Builder() {}

        /** Include a pose in the packet. */
        public Builder pose(double x, double y, double z, double yawRad) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yawRad = yawRad;
            return this;
        }

        /** Include a pose in the packet. */
        public Builder pose(Pose2d robotPose2d) {
            this.x = robotPose2d.getX();
            this.y = robotPose2d.getY();
            this.z = 0.0;
            this.yawRad = robotPose2d.getRotation().getRadians();
            return this;
        }

        /** Include instructions to capture frames in this packet. */
        public Builder capture(boolean saveInputFrame, boolean saveOutputFrame, boolean saveDepthFrame) {
            this.saveInputFrame = saveInputFrame;
            this.saveOutputFrame = saveOutputFrame;
            this.saveDepthFrame = saveDepthFrame;
            return this;
        }

        public UDPPacket build() {
            UDPPacket pkt = new UDPPacket();

            // Pose is optional, but if any pose field is set, require all.
            boolean hasAnyPose =
                    x != null || y != null || z != null || yawRad != null;

            if (hasAnyPose) {
                Objects.requireNonNull(x, "x must be set when pose() is used");
                Objects.requireNonNull(y, "y must be set when pose() is used");
                Objects.requireNonNull(z, "z must be set when pose() is used");
                Objects.requireNonNull(yawRad, "yawRad must be set when pose() is used");

                UDPPacket.Translation translation = new UDPPacket.Translation();
                translation.x = x;
                translation.y = y;
                translation.z = z;

                UDPPacket.RobotPose robotPos = new UDPPacket.RobotPose();
                robotPos.translation = translation;
                robotPos.yaw_rad = yawRad;

                pkt.robot_pose = robotPos;
            }

            // Capture is optional, but if any capture field is set, require all.
            boolean hasAnyCapture = 
                saveInputFrame != null || saveOutputFrame != null || saveDepthFrame != null;

            if (hasAnyCapture) {
                Objects.requireNonNull(saveInputFrame, "saveInputFrame must be set when capture() is used");
                Objects.requireNonNull(saveOutputFrame, "saveOutputFrame must be set when capture() is used");
                Objects.requireNonNull(saveDepthFrame, "saveDepthFrame must be set when capture() is used");

                UDPPacket.Capture capture = new UDPPacket.Capture();
                capture.saveDepthFrame = saveDepthFrame;
                capture.saveInputFrame = saveDepthFrame;
                capture.saveOutputFrame = saveOutputFrame;

                pkt.capture = capture;
            }

            return pkt;
        }
    }
}

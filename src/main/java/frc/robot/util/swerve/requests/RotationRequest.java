package frc.robot.util.swerve.requests;

import edu.wpi.first.math.geometry.Rotation2d;

public sealed interface RotationRequest
    permits RotationRequest.Stop,
            RotationRequest.Velocity,
            RotationRequest.Position {

    record Stop() implements RotationRequest {}

    record Velocity(double velocity)
        implements RotationRequest {}

    record Position(Rotation2d position, double maxSpeed)
        implements RotationRequest {

        private static final double DEFAULT_MAX_SPEED = Double.POSITIVE_INFINITY;

        public Position(Rotation2d position) {
            this(position, DEFAULT_MAX_SPEED);
        }
    }
}
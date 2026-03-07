package frc.robot.util.swerve.requests;

import edu.wpi.first.math.geometry.Translation2d;

public sealed interface TranslationRequest
    permits TranslationRequest.Stop,
            TranslationRequest.Velocity,
            TranslationRequest.Position {

    record Stop() implements TranslationRequest {}

    record Velocity(Translation2d velocity, boolean fieldRelative)
        implements TranslationRequest {}

    record Position(Translation2d position, double maxSpeed)
        implements TranslationRequest {

        private static final double DEFAULT_MAX_SPEED = Double.POSITIVE_INFINITY;

        public Position(Translation2d position) {
            this(position, DEFAULT_MAX_SPEED);
        }
    }
}
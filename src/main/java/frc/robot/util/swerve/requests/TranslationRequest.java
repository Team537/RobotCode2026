package frc.robot.util.swerve.requests;

import edu.wpi.first.math.geometry.Translation2d;

public sealed interface TranslationRequest
    permits TranslationRequest.Stop,
            TranslationRequest.Velocity,
            TranslationRequest.Position {

    record Stop() implements TranslationRequest {}

    record Velocity(Translation2d velocity, boolean fieldRelative)
        implements TranslationRequest {}

    record Position(Translation2d position)
        implements TranslationRequest {}
}
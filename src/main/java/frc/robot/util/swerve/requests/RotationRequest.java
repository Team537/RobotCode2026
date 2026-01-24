package frc.robot.util.swerve.requests;

import edu.wpi.first.math.geometry.Rotation2d;

public sealed interface RotationRequest
    permits RotationRequest.Stop,
            RotationRequest.Velocity,
            RotationRequest.Position,
            RotationRequest.ForcePosition {

    record Stop() implements RotationRequest {}

    record Velocity(double velocity)
        implements RotationRequest {}

    record Position(Rotation2d position)
        implements RotationRequest {}

    record ForcePosition(Rotation2d rotation, double forcePower) 
        implements RotationRequest {}
}

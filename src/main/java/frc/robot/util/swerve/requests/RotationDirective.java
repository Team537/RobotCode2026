package frc.robot.util.swerve.requests;

public interface RotationDirective {

    default void init() {}

    RotationRequest getRequest();

    default boolean isFinished() {
        return false;
    }

    default void end(boolean interrupted) {}
}

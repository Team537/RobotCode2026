package frc.robot.util.swerve.requests;

public interface TranslationDirective {

    default void init() {}

    TranslationRequest getRequest();

    default boolean isFinished() {
        return false;
    }

    default void end(boolean interrupted) {}
}

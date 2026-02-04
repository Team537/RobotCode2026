package frc.robot.util.field;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Utility class for querying match timing, alliance state, and hub
 * availability.
 * <p>
 * This class centralizes all logic related to:
 * <ul>
 * <li>Match time and period calculation</li>
 * <li>Alliance and advantage resolution</li>
 * <li>Hub activity state (including unknown / indeterminate states)</li>
 * <li>Time-until hub state transitions</li>
 * </ul>
 *
 * <p>
 * <b>Null conventions:</b>
 * <ul>
 * <li>{@code null} indicates that a value is currently unknown or cannot be
 * determined</li>
 * <li>{@code 0.0} indicates that a state is already true</li>
 * <li>Negative time values indicate invalid match timing</li>
 * </ul>
 *
 * <p>
 * This class is intentionally stateless and safe to call from any command.
 */
public final class FieldUtil {

    /* --------------------------------------------------------------------- */
    /* Match timing utilities */
    /* --------------------------------------------------------------------- */

    /**
     * Returns the elapsed match time in seconds since the start of the match.
     * <p>
     * During autonomous, this is time since auto start.
     * During teleop, this includes the full autonomous period.
     *
     * @return elapsed match time in seconds, or empty if unavailable
     */
    public static Optional<Double> getElapsedTime() {
        double timeLeft = Timer.getMatchTime();
        if (timeLeft < 0.0) {
            return Optional.empty();
        }

        if (DriverStation.isAutonomousEnabled()) {
            return Optional.of(Constants.Field.AUTONOMOUS_PERIOD - timeLeft);
        }

        if (DriverStation.isTeleopEnabled()) {
            return Optional.of(
                Constants.Field.TELEOP_PERIOD - timeLeft
                    + Constants.Field.AUTONOMOUS_PERIOD
            );
        }

        return Optional.empty();
    }

    /** @return true if the robot is currently in autonomous mode */
    public static boolean isAutonomous() {
        return DriverStation.isAutonomousEnabled();
    }

    /** @return true if the robot is currently in teleop mode */
    public static boolean isTeleOp() {
        return DriverStation.isTeleopEnabled();
    }

    /* --------------------------------------------------------------------- */
    /* Alliance utilities */
    /* --------------------------------------------------------------------- */

    /**
     * Returns the robot's alliance.
     *
     * @return the robot's alliance, or empty if unavailable
     */
    public static Optional<frc.robot.util.field.Alliance> getAlliance() {
        return DriverStation.getAlliance()
            .map(dsAlliance ->
                dsAlliance == DriverStation.Alliance.Red
                    ? frc.robot.util.field.Alliance.RED
                    : frc.robot.util.field.Alliance.BLUE
            );
    }

    /**
     * Returns the opposing alliance.
     *
     * @return the opposing alliance, or empty if unavailable
     */
    public static Optional<frc.robot.util.field.Alliance> getOpposingAlliance() {
        return getAlliance().map(alliance ->
            alliance == frc.robot.util.field.Alliance.RED
                ? frc.robot.util.field.Alliance.BLUE
                : frc.robot.util.field.Alliance.RED
        );
    }

    /**
     * Returns the alliance currently holding hub advantage.
     *
     * @return the advantage alliance, or empty if unavailable or invalid
     */
    public static Optional<frc.robot.util.field.Alliance> getAdvantageAlliance() {
        String data = DriverStation.getGameSpecificMessage();
        if (data.isEmpty()) {
            return Optional.empty();
        }

        return switch (data.charAt(0)) {
            case 'B' -> Optional.of(frc.robot.util.field.Alliance.BLUE);
            case 'R' -> Optional.of(frc.robot.util.field.Alliance.RED);
            default -> Optional.empty();
        };
    }

    /* --------------------------------------------------------------------- */
    /* Match period utilities */
    /* --------------------------------------------------------------------- */

    /**
     * Returns the match period corresponding to a given timestamp.
     *
     * @param timestamp elapsed match time in seconds
     * @return the matching MatchPeriod, or empty if invalid
     */
    public static Optional<MatchPeriod> getMatchPeriod(double timestamp) {
        if (timestamp < 0.0) {
            return Optional.empty();
        }

        for (MatchPeriod period : MatchPeriod.values()) {
            if (period.contains(timestamp)) {
                return Optional.of(period);
            }
        }

        return Optional.empty();
    }

    /**
     * Returns the current match period based on elapsed time.
     *
     * @return the current MatchPeriod, or empty if unavailable
     */
    public static Optional<MatchPeriod> getCurrentMatchPeriod() {
        return getElapsedTime().flatMap(FieldUtil::getMatchPeriod);
    }

    /**
     * @return the next match period after the current time, or empty if none exists
     */
    public static Optional<MatchPeriod> getNextMatchPeriod() {
        return getElapsedTime().flatMap(now -> {
            MatchPeriod next = null;

            for (MatchPeriod period : MatchPeriod.values()) {
                if (period.startTime > now) {
                    if (next == null || period.startTime < next.startTime) {
                        next = period;
                    }
                }
            }

            return Optional.ofNullable(next);
        });
    }

    /**
     * @return the previous match period before the current time, or empty if none exists
     */
    public static Optional<MatchPeriod> getPreviousMatchPeriod() {
        return getElapsedTime().flatMap(now -> {
            MatchPeriod previous = null;

            for (MatchPeriod period : MatchPeriod.values()) {
                if (period.endTime < now) {
                    if (previous == null || period.endTime > previous.endTime) {
                        previous = period;
                    }
                }
            }

            return Optional.ofNullable(previous);
        });
    }

    /**
     * Returns the time elapsed (seconds) within the current match period.
     *
     * @return elapsed seconds, or empty if unavailable
     */
    public static Optional<Double> getTimeElapsedInCurrentPeriod() {
        return getCurrentMatchPeriod().flatMap(period ->
            getElapsedTime().map(elapsed ->
                elapsed - period.startTime
            )
        );
    }

    /**
     * Returns the time remaining until the end of the current match period.
     *
     * @return remaining seconds, or empty if unavailable
     */
    public static Optional<Double> getTimeUntilNextPeriod() {
        return getCurrentMatchPeriod().flatMap(period ->
            getElapsedTime().map(elapsed ->
                period.endTime - elapsed
            )
        );
    }

    /* --------------------------------------------------------------------- */
    /* Hub state utilities */
    /* --------------------------------------------------------------------- */

    /**
     * Determines whether the hub is active for a given alliance during a specific
     * match period.
     *
     * @param period   the match period to evaluate
     * @param alliance the alliance being queried
     * @return true if active, false if inactive, or empty if unknown
     */
    public static Optional<Boolean> isHubActive(
        MatchPeriod period,
        frc.robot.util.field.Alliance alliance
    ) {

        if (period.hubRule == HubRule.BOTH) {
            return Optional.of(true);
        }

        return getAdvantageAlliance().map(advantage ->
            period.hubRule == HubRule.ADVANTAGE_ALLIANCE
                ? advantage == alliance
                : advantage != alliance
        );
    }

    /**
     * Determines whether the hub is active for a given alliance at a specific
     * timestamp.
     */
    public static Optional<Boolean> isHubActive(
        double timestamp,
        frc.robot.util.field.Alliance alliance
    ) {
        return getMatchPeriod(timestamp)
            .flatMap(period -> isHubActive(period, alliance));
    }

    /**
     * Determines whether the hub is active for the robot's alliance during the
     * current match period.
     */
    public static Optional<Boolean> isOurHubCurrentlyActive() {
        return getCurrentMatchPeriod().flatMap(period ->
            getAlliance().flatMap(alliance ->
                isHubActive(period, alliance)
            )
        );
    }

    /**
     * Determines whether the hub will be active for the robot's alliance in the
     * next match period.
     */
    public static Optional<Boolean> isOurHubActiveInNextMatchPeriod() {
        return getNextMatchPeriod().flatMap(period ->
            getAlliance().flatMap(alliance ->
                isHubActive(period, alliance)
            )
        );
    }

    /**
     * Determines whether the hub is active for the opposing alliance during the
     * current match period.
     */
    public static Optional<Boolean> isOpposingHubCurrentlyActive() {
        return getCurrentMatchPeriod().flatMap(period ->
            getOpposingAlliance().flatMap(alliance ->
                isHubActive(period, alliance)
            )
        );
    }

    /**
     * Determines whether the hub will be active for the opposing alliance in the
     * next match period.
     */
    public static Optional<Boolean> isOpposingHubActiveInNextMatchPeriod() {
        return getNextMatchPeriod().flatMap(period ->
            getOpposingAlliance().flatMap(alliance ->
                isHubActive(period, alliance)
            )
        );
    }

    /* --------------------------------------------------------------------- */
    /* Hub timing utilities */
    /* --------------------------------------------------------------------- */

    /**
     * Returns the time until the hub reaches a desired state for a given alliance.
     *
     * @param alliance     the alliance being queried
     * @param targetActive desired hub state
     * @return seconds until the state occurs, {@code 0.0} if already true,
     *         or empty if unknown or unreachable
     */
    public static Optional<Double> timeUntilHubState(
        Alliance alliance,
        boolean targetActive
    ) {
        return getElapsedTime().flatMap(now ->
            getCurrentMatchPeriod().flatMap(currentPeriod ->
                isHubActive(currentPeriod, alliance).flatMap(currentState -> {

                    // Already in desired state
                    if (currentState == targetActive) {
                        return Optional.of(0.0);
                    }

                    // Search future periods
                    for (MatchPeriod period : MatchPeriod.values()) {
                        if (period.startTime <= now) {
                            continue;
                        }

                        Optional<Boolean> state =
                            isHubActive(period, alliance);

                        if (state.isPresent() && state.get() == targetActive) {
                            return Optional.of(period.startTime - now);
                        }
                    }

                    return Optional.empty();
                })
            )
        );
    }

    /**
     * @return time until the hub becomes active for the robot's alliance,
     *         or empty if unknown or unreachable
     */
    public static Optional<Double> timeUntilHubActive() {
        return getAlliance()
            .flatMap(alliance -> timeUntilHubState(alliance, true));
    }

    /**
     * @return time until the hub becomes inactive for the robot's alliance,
     *         or empty if unknown or unreachable
     */
    public static Optional<Double> timeUntilHubInactive() {
        return getAlliance()
            .flatMap(alliance -> timeUntilHubState(alliance, false));
    }

    /**
     * @return time until the hub becomes active for the opposing alliance,
     *         or empty if unknown or unreachable
     */
    public static Optional<Double> timeUntilOpposingHubActive() {
        return getOpposingAlliance()
            .flatMap(alliance -> timeUntilHubState(alliance, true));
    }

    /**
     * @return time until the hub becomes inactive for the opposing alliance,
     *         or empty if unknown or unreachable
     */
    public static Optional<Double> timeUntilOpposingHubInactive() {
        return getOpposingAlliance()
            .flatMap(alliance -> timeUntilHubState(alliance, false));
    }
}
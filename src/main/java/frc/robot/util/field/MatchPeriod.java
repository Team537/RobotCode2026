package frc.robot.util.field;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum MatchPeriod {

    AUTO(
        "Auto",
        HubRule.BOTH,
        Constants.Field.AUTO_START,
        Constants.Field.AUTO_END
    ),

    TRANSITION(
        "Transition",
        HubRule.BOTH,
        Constants.Field.TRANSITION_START,
        Constants.Field.TRANSITION_END
    ),

    SHIFT_1(
        "Shift 1",
        HubRule.DISADVANTAGE_ALLIANCE,
        Constants.Field.SHIFT_1_START,
        Constants.Field.SHIFT_1_END
    ),

    SHIFT_2(
        "Shift 2",
        HubRule.ADVANTAGE_ALLIANCE,
        Constants.Field.SHIFT_2_START,
        Constants.Field.SHIFT_2_END
    ),

    SHIFT_3(
        "Shift 3",
        HubRule.DISADVANTAGE_ALLIANCE,
        Constants.Field.SHIFT_3_START,
        Constants.Field.SHIFT_3_END
    ),

    SHIFT_4(
        "Shift 4",
        HubRule.ADVANTAGE_ALLIANCE,
        Constants.Field.SHIFT_4_START,
        Constants.Field.SHIFT_4_END
    ),

    ENDGAME(
        "Endgame",
        HubRule.BOTH,
        Constants.Field.ENDGAME_START,
        Constants.Field.ENDGAME_END
    );

    public final String name;

    /** Rule describing which hub(s) are active */
    public final HubRule hubRule;

    /** Elapsed-time start (seconds) */
    public final double startTime;

    /** Elapsed-time end (seconds) */
    public final double endTime;

    MatchPeriod(
        String name,
        HubRule hubRule,
        double startTime,
        double endTime
    ) {
        this.name = name;
        this.hubRule = hubRule;
        this.startTime = startTime;
        this.endTime = endTime;
    }

    /** Returns true if the given elapsed time is inside this phase */
    public boolean contains(double elapsedTime) {
        return elapsedTime >= startTime && elapsedTime < endTime;
    }

}

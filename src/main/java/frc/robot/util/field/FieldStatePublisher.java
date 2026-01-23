package frc.robot.util.field;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.CommandTimeline;
import frc.robot.util.dashboard.Elastic;

public class FieldStatePublisher {

	/** Call this once per robot periodic cycle */
	public static void update() {
		/* --------------------------------------------------------------------- */
		/* Alliance info */
		/* --------------------------------------------------------------------- */

		FieldUtil.getAlliance().ifPresentOrElse(
			alliance -> SmartDashboard.putString("Alliance", alliance.name()),
			() -> SmartDashboard.putString("Alliance", "Unknown")
		);

		FieldUtil.getOpposingAlliance().ifPresentOrElse(
			alliance -> SmartDashboard.putString("Opposing Alliance", alliance.name()),
			() -> SmartDashboard.putString("Opposing Alliance", "Unknown")
		);

		FieldUtil.getAdvantageAlliance().ifPresentOrElse(
			alliance -> SmartDashboard.putString("Advantage Alliance", alliance.name()),
			() -> SmartDashboard.putString("Advantage Alliance", "Unknown")
		);

		/* --------------------------------------------------------------------- */
		/* Match timing */
		/* --------------------------------------------------------------------- */

		SmartDashboard.putNumber(
			"Elapsed Time",
			FieldUtil.getElapsedTime().orElse(0.0)
		);

		SmartDashboard.putNumber(
			"Time in Current Period",
			FieldUtil.getTimeElapsedInCurrentPeriod().orElse(0.0)
		);

		SmartDashboard.putNumber(
			"Time Until Next Period",
			FieldUtil.getTimeUntilNextPeriod().orElse(0.0)
		);

		FieldUtil.getCurrentMatchPeriod().ifPresentOrElse(
			period -> SmartDashboard.putString("Period", period.name),
			() -> SmartDashboard.putString("Period", "Unknown")
		);

		/* --------------------------------------------------------------------- */
		/* Hub state */
		/* --------------------------------------------------------------------- */

		SmartDashboard.putBoolean(
			"Our Hub Active",
			FieldUtil.isOurHubCurrentlyActive().orElse(false)
		);

		SmartDashboard.putBoolean(
			"Opposing Hub Active",
			FieldUtil.isOpposingHubCurrentlyActive().orElse(false)
		);

		SmartDashboard.putBoolean(
			"Our Hub Active In Next Period",
			FieldUtil.isOurHubActiveInNextMatchPeriod().orElse(false)
		);

		SmartDashboard.putBoolean(
			"Opposing Hub Active In Next Period",
			FieldUtil.isOpposingHubActiveInNextMatchPeriod().orElse(false)
		);

		/* --------------------------------------------------------------------- */
		/* Hub timing */
		/* --------------------------------------------------------------------- */

		SmartDashboard.putNumber(
			"Time Until Our Hub Active",
			FieldUtil.timeUntilHubActive().orElse(-.0)
		);

		SmartDashboard.putNumber(
			"Time Until Our Hub Inactive",
			FieldUtil.timeUntilHubInactive().orElse(-.0)
		);

		SmartDashboard.putNumber(
			"Time Until Opposing Hub Active",
			FieldUtil.timeUntilOpposingHubActive().orElse(0.0)
		);

		SmartDashboard.putNumber(
			"Time Until Opposing Hub Inactive",
			FieldUtil.timeUntilOpposingHubInactive().orElse(0.0)
		);
	}

	public static void setupElasticNotifications() {

		// Loop over all match periods
		for (MatchPeriod period : MatchPeriod.values()) {
			double startTime = period.startTime;

			// Notify at the start of the period
			CommandTimeline.schedule(startTime, () -> {
				String msg = String.format(
					"%s Period Begun (Duration: %.1f seconds). Our hub is %s, Opposing hub is %s.",
					period.name,
					period.endTime - period.startTime,
					FieldUtil.isHubActive(period, FieldUtil.getAlliance().orElse(Alliance.BLUE)).orElse(false) ? "ACTIVE" : "INACTIVE",
					FieldUtil.isHubActive(period, FieldUtil.getOpposingAlliance().orElse(Alliance.RED)).orElse(false) ? "ACTIVE" : "INACTIVE"
				);
				Elastic.sendNotification(new Elastic.Notification(
					Elastic.NotificationLevel.INFO,
					period.name,
					msg,
					(int) Units.secondsToMilliseconds(8.0)
				));
			});

		}

	}
}

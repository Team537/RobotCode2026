package frc.robot.commands.swerve;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.base.RebuildingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationRequest;

/**
 * Drives the robot to collect a dynamically moving target by first pathfinding
 * to a reasonable approach pose, then switching to a direct, velocity-driven
 * intake alignment phase.
 *
 * <p>The command continuously tracks the supplied target position and will
 * replan or fall back to pathfinding if the target moves too far.
 */
public class CollectTargetCommand extends SequentialCommandGroup {

    private final DriveSubsystem drive;
    private final Transform2d intakeTransform = Constants.Intake.intakeTransform;

    /** The most recent pose used for path planning, used to decide when to replan. */
    private Pose2d lastPlannedPose;

    public CollectTargetCommand(
            DriveSubsystem drive,
            Supplier<Translation2d> targetSupplier
    ) {

        this.drive = drive;

        // ------------------------------------------------------------------
        // Pathfinding Phase
        // ------------------------------------------------------------------
        // Uses a rebuilding command so that the path can be regenerated if the
        // target moves significantly while pathfinding is active.
        Command pathfindCommand = new RebuildingCommand(

            // Builds a new pathfinding command toward the intake-aligned pose
            () -> {
                Pose2d targetPose = getIntakeAlignedPose(targetSupplier.get());
                lastPlannedPose = targetPose;
                return drive.getRawPathfindToPoseCommand(targetPose, 2.0);
            },

            // Rebuild condition:
            // If the target pose has drifted too far from the last planned pose,
            // the current path is considered invalid and should be regenerated.
            () -> {
                Pose2d currentTargetPose = getIntakeAlignedPose(targetSupplier.get());
                return lastPlannedPose != null
                        && currentTargetPose.getTranslation()
                                .getDistance(
                                        lastPlannedPose.getTranslation())
                                > Constants.Drive.BALL_HUNT_REPLANNING_DISTANCE;
            },
            Set.of(drive)
        );

        // ------------------------------------------------------------------
        // Direct Drive Phase
        // ------------------------------------------------------------------
        // Once close enough, directly drive the intake into the target using
        // velocity-based translation while strictly enforcing intake heading.
        Command directDriveCommand = new RunCommand(
            () -> {
                drive.driveWithCompositeRequests(
                    // Drive toward the intake-aligned pose at a constant velocity
                    new TranslationRequest.PositionWithVelocity(
                        getIntakeAlignedPose(targetSupplier.get()).getTranslation(),
                        2.0
                    ),

                    // Force the robot's rotation to match the intake-aligned pose
                    // while scaling translation until alignment is achieved
                    new RotationRequest.ForcePosition(
                        getIntakeAlignedPose(targetSupplier.get()).getRotation()
                    )
                );
            },
            drive
        )

        // Terminate the direct-drive phase if:
        //  - The target has moved enough to justify replanning, AND
        //  - The intake is no longer within the acceptable direct-drive range
        .until(() -> {
            Pose2d currentTargetPose = getIntakeAlignedPose(targetSupplier.get());

            boolean shouldReplan =
                lastPlannedPose != null &&
                currentTargetPose.getTranslation()
                    .getDistance(lastPlannedPose.getTranslation())
                    > Constants.Drive.BALL_HUNT_REPLANNING_DISTANCE;

            boolean tooFar =
                drive.getPose().transformBy(intakeTransform).getTranslation()
                    .getDistance(targetSupplier.get())
                    > Constants.Drive.BALL_HUNT_DIRECT_DRIVE_DISTANCE;

            return shouldReplan && tooFar;
        });

        // Execute pathfinding first, then transition into direct intake driving
        addCommands(
                pathfindCommand,
                directDriveCommand
        );
    }

    /**
     * Computes a robot pose such that the intake ends up positioned exactly on
     * the target translation, with the intake oriented toward the target.
     *
     * <p>This is done by constructing the desired intake pose first, then
     * inverting the intake transform to recover the corresponding robot pose.
     */
    private Pose2d getIntakeAlignedPose(Translation2d target) {

        // Current robot pose and derived intake pose
        Pose2d robotPose = drive.getPose();
        Pose2d currentIntakePose = robotPose.transformBy(intakeTransform);

        // Compute the desired intake heading based on the direction to the target
        Rotation2d intakeRotation =
            target.minus(currentIntakePose.getTranslation()).getAngle();

        // Desired intake pose: located on the target and facing into it
        Pose2d desiredIntakePose = new Pose2d(target, intakeRotation);

        // Invert the intake transform to recover the robot pose that would
        // place the intake at the desired pose
        return desiredIntakePose.transformBy(intakeTransform.inverse());
    }
}

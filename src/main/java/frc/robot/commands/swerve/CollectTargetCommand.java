package frc.robot.commands.swerve;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final Field2d testField = new Field2d();

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
                SmartDashboard.putBoolean("Is Ball Hunt Pathfinding", true);
                Pose2d targetPose = getIntakeAlignedPose(targetSupplier.get());
                lastPlannedPose = targetPose;
                return drive.getRawPathfindToPoseCommand(targetPose, 2.0);
            },

            // Rebuild condition:
            // If the target pose has drifted too far from the last planned pose,
            // the current path is considered invalid and should be regenerated.
            () -> {
                Pose2d currentTargetPose = getIntakeAlignedPose(targetSupplier.get());
                return lastPlannedPose != null &&
                    currentTargetPose.getTranslation()
                        .getDistance(lastPlannedPose.getTranslation())
                        > Constants.Drive.BALL_HUNT_REPLANNING_DISTANCE ||
                    Math.abs(currentTargetPose.getRotation().minus(lastPlannedPose.getRotation()).getRadians()) > 0.1;
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
                SmartDashboard.putBoolean("Is Ball Hunt Pathfinding", false);
                drive.driveWithCompositeRequests(
                    // Drive toward the intake-aligned pose at a constant velocity
                    new TranslationRequest.PositionWithVelocity(
                        getIntakeAlignedPose(targetSupplier.get()).getTranslation(),
                        2.0
                    ),

                    // Force the robot's rotation to match the intake-aligned pose
                    // while scaling translation until alignment is achieved
                    new RotationRequest.ForcePosition(
                        getIntakeAlignedPose(targetSupplier.get()).getRotation(),
                        Constants.Drive.BALL_HUNT_ROTATION_FORCE_POWER
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
                    > Constants.Drive.BALL_HUNT_REPLANNING_DISTANCE ||
                Math.abs(currentTargetPose.getRotation().minus(lastPlannedPose.getRotation()).getRadians()) > 0.1;


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

        Pose2d robotPose = drive.getPose();

        // Vector from robot to target (world frame)
        Translation2d toTarget =
            target.minus(robotPose.getTranslation());

        // Desired robot rotation:
        // point at target, minus intake angular offset
        Rotation2d robotRotation =
            toTarget.getAngle()
                .minus(intakeTransform.getRotation());

        // Desired robot translation:
        // back out intake offset at that rotation
        Translation2d robotTranslation =
            target.minus(
                intakeTransform
                    .getTranslation()
                    .rotateBy(robotRotation)
            );

        Pose2d pose = new Pose2d(robotTranslation, robotRotation);

        testField.getObject("Target")
            .setPose(pose);
        SmartDashboard.putData("Target Field", testField);

        return pose;
    }

}

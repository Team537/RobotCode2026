package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.field.regions.Region3d;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationRequest;

/**
 * Command that ensures the robot enters a 3D region and then patrols inside it.
 *
 * Behavior:
 *  1. If the robot is NOT currently inside the region:
 *     - Pathfinds to a specified patrol position while maintaining current heading
 *     - Ends early once the robot enters the region
 *     - Waits inside the region for a short stabilization time
 *  2. Once inside the region:
 *     - Stops all translation
 *     - Continuously rotates at a fixed angular velocity (patrol behavior)
 */
public class PatrolRegionCommand extends SequentialCommandGroup {

    /**
     * Creates a patrol command targeting a specific position inside the region.
     *
     * @param drive           Drive subsystem
     * @param region          Region that must be entered and patrolled
     * @param patrolPosition Desired XY position to pathfind to within the region
     */
    public PatrolRegionCommand(DriveSubsystem drive, Region3d region, Translation2d patrolPosition) {

        addCommands(
            new ConditionalCommand(
                // IF we are outside the region:
                drive.getPathfindToPoseCommand(
                    // Generate a target pose using the desired patrol position
                    // while preserving the robot's current rotation
                    new Pose2d(
                        patrolPosition,
                        drive.getPose().getRotation()
                    )
                ).raceWith(
                    // End pathfinding after we are in the region for a certain amount of time
                    new WaitUntilCommand(
                        () -> region.contains(
                            toTranslation3d(drive.getPose().getTranslation())
                        )
                    ).andThen(
                        new WaitCommand(Constants.Drive.PATROL_REGION_TIME)
                    )
                ),
                // ELSE (already inside the region): do nothing and continue
                Commands.none(),
                // Condition: only pathfind if we are currently outside the region
                () -> !region.contains(
                    toTranslation3d(drive.getPose().getTranslation())
                )
            ),

            // Once inside the region, stop all translation and rotate in place
            new RunCommand(
                () -> drive.driveWithCompositeRequests(
                    new TranslationRequest.Stop(),
                    new RotationRequest.Velocity(
                        Constants.Drive.PATROL_ANGULAR_VELOCITY
                    )
                ),
                drive
            )
        );
    }

    /**
     * Convenience constructor that patrols the center of the region.
     *
     * @param drive  Drive subsystem
     * @param region Region to patrol
     */
    public PatrolRegionCommand(DriveSubsystem drive, Region3d region) {
        this(
            drive,
            region,
            region.getBounds().getCenter().toTranslation2d()
        );
    }

    /**
     * Utility helper to convert a 2D translation into a 3D translation
     * with zero Z height, allowing compatibility with Region3d checks.
     */
    private Translation3d toTranslation3d(Translation2d translation) {
        return new Translation3d(
            translation.getX(),
            translation.getY(),
            0.0
        );
    }

}

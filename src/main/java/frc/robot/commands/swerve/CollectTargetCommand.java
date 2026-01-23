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
import frc.robot.commands.base.RebuildingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationRequest;

public class CollectTargetCommand extends SequentialCommandGroup {

    private final Transform2d intakeTransform = new Transform2d(
            new Translation2d(
                    0.3,
                    0.0),
            Rotation2d.kZero);
    private final double replanningDistance = 0.2;
    private final double directDriveDistance = 1.0;

    private DriveSubsystem drive;

    private Pose2d lastPlannedPose;

    public CollectTargetCommand(
            DriveSubsystem drive,
            Supplier<Translation2d> targetSupplier) {

        this.drive = drive;

        // Pathfinding Command
        Command pathfindCommand = new RebuildingCommand(

            // Builder for the command
            () -> {
                Pose2d targetPose = getIntakeAlignedPose(targetSupplier.get());
                lastPlannedPose = targetPose;
                return drive.getRawPathfindToPoseCommand(targetPose,2.0);
            },

            // Rebuild condition (pose is too far)
            () -> {
                Pose2d currentTargetPose = getIntakeAlignedPose(targetSupplier.get());
                return lastPlannedPose != null
                        && currentTargetPose.getTranslation()
                                .getDistance(
                                        lastPlannedPose.getTranslation()) > replanningDistance;
            },
            Set.of(drive)
        );

        // Compose them sequentially
        Command directDriveCommand = new RunCommand(
            () -> {
                drive.driveWithCompositeRequests(
                    new TranslationRequest.PositionWithVelocity(getIntakeAlignedPose(targetSupplier.get()).getTranslation(),2.0),
                    new RotationRequest.ForcePosition(getIntakeAlignedPose(targetSupplier.get()).getRotation())
                );
            },
            drive
        )

            // End if the target moves significantly and it gets too far
        
        .until(() -> {
            Pose2d currentTargetPose = getIntakeAlignedPose(targetSupplier.get());
            boolean shouldReplan =
                lastPlannedPose != null &&
                currentTargetPose.getTranslation()
                    .getDistance(lastPlannedPose.getTranslation()) > replanningDistance;

            boolean tooFar =
                drive.getPose().transformBy(intakeTransform).getTranslation()
                    .getDistance(targetSupplier.get()) > directDriveDistance;

            return shouldReplan && tooFar;
        });

        addCommands(
                pathfindCommand,
                directDriveCommand);
    }

    private Pose2d getIntakeAlignedPose(Translation2d target) {

        // Current robot & intake pose
        Pose2d robotPose = drive.getPose();
        Pose2d currentIntakePose = robotPose.transformBy(intakeTransform);

        // Direction the intake should face (from current intake -> target)
        Rotation2d intakeRotation = target.minus(currentIntakePose.getTranslation()).getAngle();

        // Desired intake pose: on the target, facing toward it
        Pose2d desiredIntakePose = new Pose2d(target, intakeRotation);

        // Back out the robot pose so that:
        // robotPose + intakeTransform = desiredIntakePose
        return desiredIntakePose.transformBy(intakeTransform.inverse());
    }

}

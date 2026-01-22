package frc.robot.commands.swerve;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.field.Fuel;
import frc.robot.util.field.regions.Region3d;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationRequest;

public class BallHuntCommand extends Command {

    private final double replanningDistance = 0.2;
    private final double directDriveDistance = 1.0;
    private final Rotation2d intakeAngle = Rotation2d.kZero;
    private final double searchAngularVelocity = 1.0;

    private final DriveSubsystem drive;
    private final Supplier<List<Fuel>> fuelsSupplier;
    private final Region3d huntingRegion;
    private final Translation2d regionCenter;

    private Command pathfindingCommand;
    private Pose2d lastTargetPose;

    public BallHuntCommand(DriveSubsystem drive, Supplier<List<Fuel>> fuelsSupplier, Region3d huntingRegion) {
        this.drive = drive;
        this.fuelsSupplier = fuelsSupplier;
        this.huntingRegion = huntingRegion;
        this.regionCenter = huntingRegion.getBounds().getCenter().toTranslation2d();

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();

        // Find the best fuel in the hunting region
        List<Fuel> fuels = fuelsSupplier.get();
        Optional<Fuel> bestFuel = fuels.stream()
                .filter(fuel -> huntingRegion.contains(fuel.getTranslation()))
                .findFirst();

        if (bestFuel.isPresent()) {
            Translation2d fuelTranslation = bestFuel.get().getTranslation().toTranslation2d();

            // Decide whether to pathfind or direct-drive
            double distanceToFuel = robotPose.getTranslation().getDistance(fuelTranslation);
            boolean pathFinished = pathfindingCommand != null && pathfindingCommand.isFinished();

            if (distanceToFuel < directDriveDistance || pathFinished) {
                // Cancel pathfinding and drive directly
                if (pathfindingCommand != null) {
                    pathfindingCommand.end(true);
                    pathfindingCommand = null;
                }

                Rotation2d angleToFuel = fuelTranslation.minus(robotPose.getTranslation()).getAngle().minus(intakeAngle);
                drive.driveToPosition(new Pose2d(fuelTranslation, angleToFuel));

            } else {
                // Use pathfinding toward the fuel
                setupPathfindingTo(fuelTranslation, robotPose);
                pathfindingCommand.execute();
            }

        } else {
            // No fuel detected
            if (huntingRegion.contains(toTranslation3d(robotPose.getTranslation()))) {
                // Scan in place
                drive.driveWithCompositeRequests(
                        new TranslationRequest.Position(robotPose.getTranslation()),
                        new RotationRequest.Velocity(searchAngularVelocity)
                );
            } else {
                // Outside hunting region: pathfind to region center
                setupPathfindingTo(regionCenter, robotPose);
                pathfindingCommand.execute();
            }
        }
    }

    /**
     * Prepares the pathfinding command toward a target translation.
     * Reuses pathfindingCommand if close enough to previous target; otherwise
     * reinitializes it.
     */
    private void setupPathfindingTo(Translation2d targetTranslation, Pose2d robotPose) {
        boolean needNewPath = pathfindingCommand == null
                || lastTargetPose == null
                || lastTargetPose.getTranslation().getDistance(targetTranslation) > replanningDistance
                || pathfindingCommand.isFinished();

        if (needNewPath) {
            if (pathfindingCommand != null) {
                pathfindingCommand.end(true);
            }

            Rotation2d angleToTarget = targetTranslation.minus(robotPose.getTranslation()).getAngle().minus(intakeAngle);
            lastTargetPose = new Pose2d(targetTranslation, angleToTarget);
            pathfindingCommand = drive.getPathfindToPoseCommand(lastTargetPose);
            pathfindingCommand.initialize();
        }
    }

    private Translation3d toTranslation3d(Translation2d translation) {
        return new Translation3d(translation.getX(), translation.getY(), 0.0);
    }

}

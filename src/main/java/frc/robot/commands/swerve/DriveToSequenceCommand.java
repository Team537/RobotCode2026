package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that drives the robot through a sequence of target poses.
 * 
 * <p>The robot will drive to each pose in order, waiting for each position
 * to be reached (within tolerance) before moving to the next one.
 */
public class DriveToSequenceCommand extends SequentialCommandGroup {

    /**
     * Creates a command that drives to a sequence of poses.
     * 
     * @param driveSubsystem The drive subsystem to use
     * @param poses Variable number of poses to drive to, in order
     */
    public DriveToSequenceCommand(DriveSubsystem driveSubsystem, Pose2d... poses) {
        this(driveSubsystem, Arrays.asList(poses));
    }

    /**
     * Creates a command that drives to a sequence of poses.
     * 
     * @param driveSubsystem The drive subsystem to use
     * @param poses List of poses to drive to, in order
     */
    public DriveToSequenceCommand(DriveSubsystem driveSubsystem, List<Pose2d> poses) {
        // Add each drive-to-pose command to the sequence
        for (Pose2d pose : poses) {
            addCommands(driveSubsystem.getPathfindToPoseCommand(pose,1.0,Rotation2d.fromDegrees(50)));
        }
        
        addRequirements(driveSubsystem);
    }

    /**
     * Builder class for creating drive sequences with a fluent API.
     */
    public static class Builder {
        private final DriveSubsystem driveSubsystem;
        private final List<Pose2d> poses;

        public Builder(DriveSubsystem driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            this.poses = new ArrayList<>();
        }

        /**
         * Adds a pose to the sequence.
         */
        public Builder addPose(Pose2d pose) {
            poses.add(pose);
            return this;
        }

        /**
         * Adds a pose with x, y coordinates and rotation to the sequence.
         */
        public Builder addPose(double x, double y, double rotationDegrees) {
            poses.add(new Pose2d(x, y, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(rotationDegrees)));
            return this;
        }

        /**
         * Builds the command.
         */
        public DriveToSequenceCommand build() {
            return new DriveToSequenceCommand(driveSubsystem, poses);
        }
    }
}

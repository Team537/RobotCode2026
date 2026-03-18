package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakeAndShootWhileDriving extends SequentialCommandGroup {

    public IntakeAndShootWhileDriving(
            DriveSubsystem drive,
            IntakePivotSubsystem intakePivot,
            IntakeRollerSubsystem intakeRoller,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            TransferSubsystem transfer,
            Supplier<Translation3d> targetingSupplier,
            Pose2d readyPose,
            Pose2d intakePose,
            boolean stowTurret,
            double maxDriveSpeed,
            double postStopDelay) {

        Command pathToReady = drive.getPathfindToPoseCommand(readyPose);

        if (stowTurret) {
            pathToReady = pathToReady.alongWith(
                    turret.getStowCommand());
        }

        addCommands(

            // Drive to ready pose
            pathToReady,

            // Intake + shoot while slowly driving
            Commands.deadline(
                drive.getDriveToPoseCommand(
                        () -> intakePose,
                        Constants.Drive.TRANSLATIONAL_TOLERANCE,
                        Constants.Drive.ROTATIONAL_TOLERANCE,
                        maxDriveSpeed,
                        Double.MAX_VALUE,
                        true),

                intakePivot.deployIntakeCommand(),
                intakeRoller.getIntakeCommand(),

                shooter.getTargetCommand(
                        targetingSupplier,
                        drive::getPose,
                        drive::getVelocity),

                transfer.getLoadCommand()
            ),

            // Stop driving + intake
            Commands.parallel(
                drive.getStopCommand(),
                intakePivot.raiseIntakeCommand(),
                intakeRoller.getStopCommand()
            ),

            // Wait before stopping shooter/turret
            Commands.waitSeconds(postStopDelay),

            // Finally stop shooter + turret
            Commands.parallel(
                shooter.getStopCommand(),
                transfer.getStopCommand()
            )
        );
    }
}
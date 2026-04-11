package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.turret.TargetingData;

public class ShootPreloadCommand extends SequentialCommandGroup {

    public ShootPreloadCommand(
        ShooterSubsystem shooter,
        TurretSubsystem turret,
        TransferSubsystem transfer,
        IntakePivotSubsystem intakePivot,
        IntakeRollerSubsystem intakeRoller,
        Supplier<TargetingData> targetSupplier,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier,
        double shootPreloadTime
    ) {

        addCommands(

            Commands.deadline(
                Commands.waitSeconds(shootPreloadTime + 2),
                Commands.parallel(
                    new ParallelCommandGroup(
                        turret.getTargetCommand(targetSupplier, robotPoseSupplier, robotVelocitySupplier),
                        shooter.getTargetCommand(targetSupplier, robotPoseSupplier, robotVelocitySupplier),
                        intakeRoller.getIntakeCommand(),
                        intakePivot.deployIntakeCommand()
                    ),
                    Commands.sequence(
                        new WaitCommand(2),
                        transfer.getLoadCommand()
                    )
                )
            ),

            Commands.parallel(
                shooter.getStopCommand(),
                transfer.getStopCommand(),
                intakePivot.raiseIntakeCommand(),
                intakeRoller.getStopCommand()
            ).withTimeout(3.0)

        );

    }

}

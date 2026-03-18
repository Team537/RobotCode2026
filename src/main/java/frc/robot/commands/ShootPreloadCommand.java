package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootPreloadCommand extends SequentialCommandGroup {

    public ShootPreloadCommand(
        ShooterSubsystem shooter,
        TurretSubsystem turret,
        TransferSubsystem transfer,
        Supplier<Translation3d> targetSupplier,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier,
        double shootPreloadTime
    ) {

        addCommands(

            Commands.deadline(
                Commands.waitSeconds(shootPreloadTime),
                turret.getTargetCommand(targetSupplier, robotPoseSupplier, robotVelocitySupplier),
                shooter.getTargetCommand(targetSupplier, robotPoseSupplier, robotVelocitySupplier),
                transfer.getLoadCommand()
            ),

            Commands.parallel(
                shooter.getStopCommand(),
                transfer.getStopCommand()
            )

        );

    }

}

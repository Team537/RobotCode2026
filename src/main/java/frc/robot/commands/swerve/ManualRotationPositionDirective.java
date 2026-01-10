package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.RotationDirective;
import frc.robot.util.swerve.requests.RotationRequest;

public class ManualRotationPositionDirective implements RotationDirective{
    
    private final DriveSubsystem drive;

    private final Supplier<Double> thetaSupplier;

    private final Rotation2d driverRotation;

    public ManualRotationPositionDirective(
        DriveSubsystem drive,
        Supplier<Double> thetaSupplier,
        Rotation2d driverRotation
    ) {
        this.drive = drive;
        this.thetaSupplier = thetaSupplier;
        this.driverRotation = driverRotation;
    }

    @Override
    public RotationRequest getRequest() {
        
        Rotation2d position = Rotation2d.fromRadians(thetaSupplier.get()).plus(driverRotation);

        return new RotationRequest.Position(position);

    }

}

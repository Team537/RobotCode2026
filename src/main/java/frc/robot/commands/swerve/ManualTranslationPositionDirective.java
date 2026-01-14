package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.TranslationDirective;
import frc.robot.util.swerve.requests.TranslationRequest;

public class ManualTranslationPositionDirective implements TranslationDirective{
    
    private final DriveSubsystem drive;

    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    private final double radius;
    private final Rotation2d driverRotation;

    Translation2d center;

    public ManualTranslationPositionDirective(
        DriveSubsystem drive,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier,
        double radius,
        Rotation2d driverRotation
    ) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.radius = radius;
        this.driverRotation = driverRotation;
    }

    @Override 
    public void init() {

        Translation2d robotTranslation = drive.getPose().getTranslation();

        center = robotTranslation.minus(getCenterOffset());

    }

    @Override
    public TranslationRequest getRequest() {
        
        Translation2d offset = getCenterOffset();

        return new TranslationRequest.Position(center.plus(offset));

    }

    private Translation2d getCenterOffset() {

        Translation2d offset = new Translation2d(
            xSupplier.get(),
            ySupplier.get()
        );

        offset = offset.times(radius);
        offset = offset.rotateBy(driverRotation);

        return offset;

    }

}

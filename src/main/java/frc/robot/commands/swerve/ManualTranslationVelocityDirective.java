package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.TranslationDirective;
import frc.robot.util.swerve.requests.TranslationRequest;

public class ManualTranslationVelocityDirective implements TranslationDirective {

    private final DriveSubsystem drive;

    private final Supplier<Double> vxSupplier;
    private final Supplier<Double> vySupplier;
    private final Supplier<Double> throttleSupplier;
    private final Supplier<Double> slowSupplier;
    private final Supplier<Boolean> fieldRelativeSupplier;

    private final double inputCurvePower;
    private final double maxNormalSpeed;
    private final double maxThrottleSpeed;
    private final double maxSlowSpeed;
    private final Rotation2d driverRotation;

    public ManualTranslationVelocityDirective(
            DriveSubsystem drive,
            Supplier<Double> vxSupplier,
            Supplier<Double> vySupplier,
            Supplier<Double> throttleSupplier,
            Supplier<Double> slowSupplier,
            Supplier<Boolean> fieldRelativeSupplier,
            double inputCurvePower,
            double maxNormalSpeed,
            double maxThrottleSpeed,
            double maxSlowSpeed,
            Rotation2d driverRotation
    ) {
        this.drive = drive;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.throttleSupplier = throttleSupplier;
        this.slowSupplier = slowSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.inputCurvePower = inputCurvePower;
        this.maxNormalSpeed = maxNormalSpeed;
        this.maxThrottleSpeed = maxThrottleSpeed;
        this.maxSlowSpeed = maxSlowSpeed;
        this.driverRotation = driverRotation;
    }

    @Override
    public TranslationRequest getRequest() {
        // --- Raw inputs ---
        double vx = vxSupplier.get();
        double vy = vySupplier.get();
        double throttle = throttleSupplier.get();
        double slow = slowSupplier.get();
        boolean fieldRelative = fieldRelativeSupplier.get();

        // --- Compute magnitude and direction ---
        double mag = Math.hypot(vx, vy);
        double angle = Math.atan2(vy, vx);

        // Clamp and Apply input curve
        mag = Math.min(mag,1.0);
        mag = Math.pow(mag, inputCurvePower);

        // Compute linear speed based on throttle/slow scaling
        double speed = maxNormalSpeed
                + throttle * (maxThrottleSpeed - maxNormalSpeed)
                + slow * (maxSlowSpeed - maxNormalSpeed);

        double finalMag = mag * speed;

        // Apply angle offsets
        if (fieldRelative) {
            angle += driverRotation.getRadians();
        }

        // Convert back to x/y components
        double finalVx = finalMag * Math.cos(angle);
        double finalVy = finalMag * Math.sin(angle);

        return new TranslationRequest.Velocity(new Translation2d(finalVx, finalVy), fieldRelative);
    }
}

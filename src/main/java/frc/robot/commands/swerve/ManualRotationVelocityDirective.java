package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.RotationDirective;
import frc.robot.util.swerve.requests.RotationRequest;

public class ManualRotationVelocityDirective implements RotationDirective {

    private final DriveSubsystem drive;

    private final Supplier<Double> omegaSupplier;
    private final Supplier<Double> throttleSupplier;
    private final Supplier<Double> slowSupplier;

    private final double inputCurvePower;
    private final double maxNormalSpeed;
    private final double maxThrottleSpeed;
    private final double maxSlowSpeed;

    public ManualRotationVelocityDirective(
            DriveSubsystem drive,
            Supplier<Double> omegaSupplier,
            Supplier<Double> throttleSupplier,
            Supplier<Double> slowSupplier,
            double inputCurvePower,
            double maxNormalSpeed,
            double maxThrottleSpeed,
            double maxSlowSpeed
    ) {
        this.drive = drive;
        this.omegaSupplier = omegaSupplier;
        this.throttleSupplier = throttleSupplier;
        this.slowSupplier = slowSupplier;
        this.inputCurvePower = inputCurvePower;
        this.maxNormalSpeed = maxNormalSpeed;
        this.maxThrottleSpeed = maxThrottleSpeed;
        this.maxSlowSpeed = maxSlowSpeed;
    }

    @Override
    public RotationRequest getRequest() {
        // --- Raw inputs ---
        double omega = omegaSupplier.get();
        double throttle = throttleSupplier.get();
        double slow = slowSupplier.get();

        // Clamp and apply input curve
        omega = MathUtil.clamp(omega,-1.0,1.0);
        omega = Math.signum(omega) * Math.pow(Math.abs(omega),inputCurvePower);

        // Compute linear speed based on throttle/slow scaling
        double speed = maxNormalSpeed
                + throttle * (maxThrottleSpeed - maxNormalSpeed)
                + slow * (maxSlowSpeed - maxNormalSpeed);

        double finalOmega = omega * speed;

        // Convert back to x/y components


        return new RotationRequest.Velocity(finalOmega);
    }
}

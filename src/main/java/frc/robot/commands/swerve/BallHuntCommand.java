package frc.robot.commands.swerve;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.field.Fuel;
import frc.robot.util.field.regions.Region3d;

/**
 * High-level autonomous behavior that continuously searches for and collects
 * fuel elements within a defined region.
 *
 * <p>The command dynamically switches between a "search" behavior and a
 * "collect target" behavior based on whether a valid target is currently
 * detected. The best target is re-evaluated every cycle, allowing the robot
 * to react to newly detected or removed fuels.
 *
 * <p>This command never finishes on its own and is intended to be interrupted
 * by higher-level autonomous logic or operator control.
 */
public class BallHuntCommand extends Command {

    private final DriveSubsystem drive;
    private final Supplier<List<Fuel>> fuelsSupplier;
    private final Region3d huntingRegion;
    private final Supplier<Command> searchCommandSupplier;

    /** The currently active sub-behavior (searching or collecting). */
    private Command activeCommand;

    /** Tracks whether a valid target existed on the previous cycle. */
    private boolean hasTarget = false;
    private Fuel bestFuel;

    public BallHuntCommand(
            DriveSubsystem drive,
            Supplier<List<Fuel>> fuelsSupplier,
            Region3d huntingRegion,
            Supplier<Command> searchCommandSupplier
    ) {
        this.drive = drive;
        this.fuelsSupplier = fuelsSupplier;
        this.huntingRegion = huntingRegion;
        this.searchCommandSupplier = searchCommandSupplier;

        // Ball hunt owns the drivetrain while active
        addRequirements(drive);
    }

    /**
     * Convenience constructor that uses a default region patrol behavior when
     * no target is available.
     */
    public BallHuntCommand(
            DriveSubsystem drive,
            Supplier<List<Fuel>> fuelsSupplier,
            Region3d huntingRegion
    ) {
        this(
            drive,
            fuelsSupplier,
            huntingRegion,
            () -> new PatrolRegionCommand(drive, huntingRegion)
        );
    }

    @Override
    public void initialize() {
        // Begin in search mode until a valid target is detected
        activeCommand = searchCommandSupplier.get().repeatedly();
        activeCommand.initialize();
        initScoringWeights();
    }

    @Override
    public void execute() {

        updateScoringWeights();

        Optional<Fuel> bestFuel = findBestFuel();

        // Detect transitions between "no target" and "target available"
        if (bestFuel.isPresent() != hasTarget) {
            hasTarget = bestFuel.isPresent();

            // Cleanly terminate the previously active behavior
            activeCommand.end(true);

            if (hasTarget) {
                // Switch to a repeated collection behavior that continuously
                // re-evaluates the best target each cycle
                this.bestFuel = bestFuel.get();
                activeCommand =
                    new CollectTargetCommand(
                        drive,
                        () -> this.bestFuel
                                .getTranslation()
                                .toTranslation2d()
                    ).repeatedly();
            } else {
                // Fall back to the search behavior when no valid targets exist
                activeCommand = searchCommandSupplier.get().repeatedly();
            }

            activeCommand.initialize();
        }

        // Manually forward execution to the active sub-command

        if (bestFuel.isPresent()) {
            this.bestFuel = bestFuel.get();
        }
        activeCommand.execute();
    }

    /**
     * Selects the best available fuel within the hunting region.
     *
     * <p>The fuels are assumed to be pre-sorted by priority, so the first
     * matching element is treated as the best candidate.
     */
    private Optional<Fuel> findBestFuel() {
        return fuelsSupplier.get().stream()
            .filter(fuel -> huntingRegion.contains(fuel.getTranslation()))
            .max(Comparator.comparingDouble(this::scoreFuel));
    }

    /**
     * Computes a heuristic desirability score for a fuel.
     *
     * <p>Higher scores indicate fuels that are closer, better aligned with the
     * intake, already being moved toward by the robot's current motion, and
     * detected with higher confidence.
     */
    private double scoreFuel(Fuel fuel) {

        Translation2d fuelTranslation = fuel.getTranslation().toTranslation2d();

        Pose2d intakePose =
            drive.getPose().transformBy(Constants.Intake.intakeTransform);

        // Vector from intake to fuel
        Translation2d translationToFuel =
            fuelTranslation.minus(intakePose.getTranslation());

        // Angular difference between intake heading and fuel direction
        Rotation2d angularError =
            translationToFuel.getAngle().minus(intakePose.getRotation());

        Translation2d robotVelocity = new Translation2d(
            drive.getVelocity().vxMetersPerSecond,
            drive.getVelocity().vyMetersPerSecond
        );

        double score = 0.0;

        // Prefer closer fuels
        double distance = translationToFuel.getNorm();
        score += -distance * translationDistanceWeight;

        // Prefer fuels aligned with the intake direction
        score += -Math.abs(angularError.getRadians())
            * rotationDistanceWeight;

        // Prefer fuels the robot is already moving toward
        if (distance > 1e-6) {
            Translation2d directionToFuel = translationToFuel.div(distance);
            double translationalVelocityScore =
                robotVelocity.dot(directionToFuel);
            score += translationalVelocityScore
                * translationalVelocityWeight;
        }

        // Prefer fuels that match current rotational motion
        double rotationalVelocityScore =
            drive.getVelocity().omegaRadiansPerSecond
                * Math.signum(angularError.getRadians());
        score += rotationalVelocityScore
            * rotationalVelocityWeight;

        // Prefer high-confidence detections
        score += fuel.getConfidence()
            * confidenceWeight;

        return score;
    }

    @Override
    public boolean isFinished() {
        // Ball hunt is a persistent behavior and must be externally canceled
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the currently active sub-command is properly terminated
        if (activeCommand != null) {
            activeCommand.end(interrupted);
        }
    }


    //TEST STUFF DELETE BEFORE MERGE LATER

    private double translationDistanceWeight;
    private double rotationDistanceWeight;
    private double translationalVelocityWeight;
    private double rotationalVelocityWeight;
    private double confidenceWeight;

    private void initScoringWeights() {
        SmartDashboard.putNumber("FuelScore/TranslationDistanceWeight", 0.5);
        SmartDashboard.putNumber("FuelScore/RotationDistanceWeight", 1.0);
        SmartDashboard.putNumber("FuelScore/TranslationalVelocityWeight", 0.5);
        SmartDashboard.putNumber("FuelScore/RotationalVelocityWeight", 0.5);
        SmartDashboard.putNumber("FuelScore/ConfidenceWeight", 1.0);
    }

    private void updateScoringWeights() {
        translationDistanceWeight =
            SmartDashboard.getNumber("FuelScore/TranslationDistanceWeight", 0.5);
        rotationDistanceWeight =
            SmartDashboard.getNumber("FuelScore/RotationDistanceWeight", 1.0);
        translationalVelocityWeight =
            SmartDashboard.getNumber("FuelScore/TranslationalVelocityWeight", 0.5);
        rotationalVelocityWeight =
            SmartDashboard.getNumber("FuelScore/RotationalVelocityWeight", 0.5);
        confidenceWeight =
            SmartDashboard.getNumber("FuelScore/ConfidenceWeight", 1.0);
    }

}

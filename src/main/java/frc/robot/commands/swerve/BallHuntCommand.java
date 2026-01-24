package frc.robot.commands.swerve;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
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
    }

    @Override
    public void execute() {
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
        return Optional.ofNullable(
            fuelsSupplier.get().stream()
                .filter(fuel ->
                    huntingRegion.contains(fuel.getTranslation())
                )
                .findFirst()
                .orElse(null)
        );
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
}

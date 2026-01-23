package frc.robot.commands.swerve;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.field.Fuel;
import frc.robot.util.field.regions.Region3d;

public class BallHuntCommand extends Command {

    private final DriveSubsystem drive;
    private final Supplier<List<Fuel>> fuelsSupplier;
    private final Region3d huntingRegion;
    private final Command searchBehavior;

    private Command activeCommand;
    private boolean hasTarget = false;

    public BallHuntCommand(
            DriveSubsystem drive,
            Supplier<List<Fuel>> fuelsSupplier,
            Region3d huntingRegion,
            Command searchBehavior
    ) {
        this.drive = drive;
        this.fuelsSupplier = fuelsSupplier;
        this.huntingRegion = huntingRegion;
        this.searchBehavior = searchBehavior;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        activeCommand = searchBehavior;
        activeCommand.initialize();
    }

    @Override
    public void execute() {
        Optional<Fuel> bestFuel = findBestFuel();

        if (bestFuel.isPresent() != hasTarget) {
            hasTarget = bestFuel.isPresent();

            activeCommand.end(true);
             if (hasTarget) {
                activeCommand = new CollectTargetCommand(drive, () -> findBestFuel().orElse(null).getTranslation().toTranslation2d()).repeatedly();
            } else {
                activeCommand = searchBehavior.repeatedly();
            }
            activeCommand.initialize();
        }

        activeCommand.execute();
    }

    private Optional<Fuel> findBestFuel() {
        return Optional.ofNullable(fuelsSupplier.get().stream()
                .filter(fuel -> huntingRegion.contains(
                        fuel.getTranslation()))
                .findFirst()
                .orElse(null));
    }

    @Override
    public boolean isFinished() {
        return false; // Ball hunt runs until canceled
    }

    @Override
    public void end(boolean interrupted) {
        if (activeCommand != null) {
            activeCommand.end(interrupted);
        }
    }
}

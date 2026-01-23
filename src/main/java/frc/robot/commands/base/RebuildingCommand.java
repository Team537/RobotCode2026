package frc.robot.commands.base;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RebuildingCommand extends Command {

    private final Supplier<Command> commandBuilder;
    private final BooleanSupplier shouldRebuild;

    private Command currentCommand;

    /**
     * @param commandBuilder builds a NEW command whenever called
     * @param shouldRebuild edge-triggered condition to rebuild the command
     * @param requirements ALL subsystems that ANY internal command might use
     */
    public RebuildingCommand(
            Supplier<Command> commandBuilder,
            BooleanSupplier shouldRebuild,
            Set<Subsystem> requirements
    ) {
        this.commandBuilder = commandBuilder;
        this.shouldRebuild = shouldRebuild;

        addRequirements(requirements.toArray(new Subsystem[0]));
    }

    @Override
    public void initialize() {
        currentCommand = commandBuilder.get();
        currentCommand.initialize();
    }

    @Override
    public void execute() {
        if (shouldRebuild.getAsBoolean()) {
            // Interrupt old command
            currentCommand.end(true);

            // Build and start new command
            currentCommand = commandBuilder.get();
            currentCommand.initialize();
        }

        currentCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
    }
}


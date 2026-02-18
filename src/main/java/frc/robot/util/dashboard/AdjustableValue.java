package frc.robot.util.dashboard;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class AdjustableValue<T extends Number> extends SubsystemBase implements Supplier<T> {
    
    protected T value;
    protected final T min;
    protected final T max;
    protected final String key;

    public AdjustableValue(String key, T initial, T min, T max) {
        this.key = key;
        this.value = initial;
        this.min = min;
        this.max = max;

        pushToDashboard();
    }

    @Override
    public T get() {
        return value;
    }

    @Override
    public void periodic() {
        updateFromDashboard();
    }

    public void set(T newValue) {
        value = clamp(newValue);
        pushToDashboard();
    }

    public void add(T delta) {
        value = clamp(addInternal(value, delta));
        pushToDashboard();
    }

    public void updateFromDashboard() {
        T dashValue = readFromDashboard();
        if (dashValue != null) {
            value = clamp(dashValue);
        }
    }

    public Command getHeldIntervalCommand(T increment, double delayTime) {

        return new SequentialCommandGroup(
            new InstantCommand(() -> add(increment)),
            new WaitCommand(delayTime),
            new RunCommand(() ->  add(increment))
        );

    }

    protected abstract T clamp(T val);
    protected abstract T addInternal(T a, T b);
    protected abstract T readFromDashboard();
    protected abstract void pushToDashboard();
}

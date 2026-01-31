package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.field.FieldUtil;

/**
 * CommandTimeline
 *
 * A simple time-based scheduler for WPILib commands.
 * Allows commands to be scheduled at specific timestamps
 * relative to the start of Autonomous or Teleop.
 *
 * This class must be updated once per robot loop.
 */
public class CommandTimeline {

    /**
     * Represents a single scheduled event in the timeline.
     */
    private static class Event {

        // Time (seconds since mode start) when this command should run
        public final double timestamp;

        // Command to schedule
        public final Command command;

        public Event(double timestamp, Command command) {
            this.timestamp = timestamp;
            this.command = command;
        }
    }

    // Separate timelines for autonomous and teleop
    private static final List<Event> events = new ArrayList<>();

    /**
     * Schedule the specified commands at a specific time
     * @param timestamp the timestamp to schedule the commands at
     * @param commands the commands to schedule at the timestamp
     */
    public static void schedule(double timestamp, Command... commands) {
        for (Command command : commands) {
            events.add(new Event(timestamp, command));
        }
    }

    /**
     * Run the specified actions at a specific time
     * @param timestamp the timestamp to run the actions at
     * @param runnables the actions to run at the timestamp
     */
    public static void schedule(double timestamp, Runnable... runnables) {
        for (Runnable runnable : runnables) {
            events.add(new Event(timestamp, new InstantCommand(runnable)));
        }
    }

    /**
     * Clears all events from the timeline
     */
    public static void cancelAll() {
        events.clear();
    }

    /**
     * Core update method.
     *
     * This must be called once per loop (autonomousPeriodic / teleopPeriodic).
     * It checks the current match time and schedules any commands
     * whose timestamps have been reached.
     */
    public static void run() {

        // Remaining time in the current match period (seconds)
        double elapsedTime = FieldUtil.getElapsedTime().orElse(0.0);

        // Ignore invalid match time (before start or after end)
        if (elapsedTime < 0.0) {
            return;
        }

        // Schedule any events whose timestamp has been reached, and mark them for removal
        List<Event> scheduledEvents = new ArrayList<>();
        for (Event event : events) {

            if (elapsedTime >= event.timestamp) {
                CommandScheduler.getInstance().schedule(event.command);
                scheduledEvents.add(event);
            }
        }

        // Remove all scheduled events
        for (Event scheduledEvent : scheduledEvents) {
            events.remove(scheduledEvent);
        }
    }
}

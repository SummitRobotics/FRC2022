// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Thread.State;

/**
 * Command which implements Threading.
 * This allows commands to be run more than every 20ms
 */
public class CommandThreader extends CommandBase {

    private volatile Command command;
    private final int period;
    private final int priority;

    private Executor executor;

    /**
     * wraps a command inside a thread allowing it to be executed independently of the scheduler.
     * it is important to not use the command in any way after it is passed
     *
     * @param command  the command to be wrapped
     * @param period   the period in ms for the command to be run at
     * @param priority the java thread priority for the command from 1-10
     *
     * @apiNote WARNING all setter methods the command calls must be
     *     synchronized or ONLY EVER used by the command or bad things will happen
     */
    public CommandThreader(Command command, double period, int priority) {
        this.command = command;
        // converts from ms to ns
        this.period = (int) (period * 1_000_000);
        this.priority = priority;

        // addRequirements is dumb and will not take in a set and get requirements
        // returns a set so this sets it directly
        this.m_requirements = command.getRequirements();
    }

    // init the command and then starts the thread
    @Override
    public void initialize() {
        command.initialize();

        executor = new Executor(command, period);
        executor.setName(Command.class.getName() + " execution thread");
        // 1-10
        executor.setPriority(priority);
        executor.start();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!isFinished()) {
            // tells the thread to stop
            executor.interrupt();

            boolean stopped = isFinished();

            // waits for the thread to stop
            while (!stopped) {
                stopped = isFinished();
            }
        }

        // ends the command
        command.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // command ends if thread stops
        return executor.getState() == State.TERMINATED;
    }

    /**
     * Executor class which handles the Thread stuff.
     */
    static class Executor extends Thread {
        private volatile Command command;
        private final int period;

        protected Executor(Command command, int period) {
            this.command = command;
            this.period = period;
        }

        @Override
        // gets called when thread starts
        public void run() {
            super.run();

            while (!command.isFinished() && !isInterrupted()) {
                // gets the system time
                long commandStartingTime = System.nanoTime();

                // executes the command
                command.execute();

                try {
                    // sleeps the thread the remaining time so that the execution period is
                    // consistent
                    long executeTime = (System.nanoTime() - commandStartingTime);
                    long sleepPeriod = period - executeTime;

                    if (sleepPeriod < 0) {
                        // print out if the thread overran
                        System.out.printf(
                                "Loop overran by %f ms%n",
                                Math.abs(((double) (sleepPeriod)) / 1_000_000));

                    } else {
                        // sleeps the thread for the calculated time
                        sleep(sleepPeriod / 1_000_000, (int) (sleepPeriod % 1_000_000));
                    }

                } catch (InterruptedException e) {
                    // will happen if the command is interrupted and needs to end early
                    // no need to handle
                    System.out.println(getName() + " was interrupted");
                }
            }
        }
    }
}


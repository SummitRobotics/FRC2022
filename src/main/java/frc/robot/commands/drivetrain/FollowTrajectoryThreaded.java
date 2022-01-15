// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;

//TODO move this to a command that is threded with commandThreder
public class FollowTrajectoryThreaded extends CommandBase {

    private Trajectory trajectory;
    private Drivetrain drivetrain;
    private Thread executionThread;
    private int period;
    private ThreadedSplineExecutor sin;

    private RamseteCommand command;

    private LEDCall splineLEDs = new LEDCall(LEDPriorities.splines, LEDRange.All).sine(Colors.Purple);

    /**
     * command to folow a trejectory object that has been saved to the roborio with threading to make it more precice
     * @param drivetrain drivetain to control
     * @param path path to the saved SerialisableMultiGearTrejectory object
     */
    public FollowTrajectoryThreaded(Drivetrain drivetrain, Trajectory trajectory) {
        super();

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        this.period = 5_000_000;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        splineLEDs.activate();

        command = new RamseteCommand(
            trajectory, 
            drivetrain::getPose,
            new RamseteController(1.5, 0.8), Drivetrain.DriveKinimatics,
            drivetrain::setMotorTargetSpeed, drivetrain
        );

        drivetrain.setPose(trajectory.getInitialPose());
        
        command.initialize();
        System.out.println("command initialized");

        // we CAN NOT touch the command outside of the thread once it has started
        // nor can we touch the drivetrain but that should be ok beacuse the scedular should handle that
        sin = new ThreadedSplineExecutor(command, drivetrain, period);
        executionThread = new Thread(sin);
        executionThread.setName("spline thread");
        //1-10
        executionThread.setPriority(10);
        executionThread.start();
        System.out.println("thread started");
    }

    @Override
    public boolean isFinished() {
        //checks if thread is running or ended
        return executionThread.getState() == State.TERMINATED;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // tells the thread to stop
            sin.stopRunning();
        }

        boolean stopped = executionThread.getState() == State.TERMINATED;

        // makes sure thread is stopped before allowing scheduler to continue to prevent unintentional movement
        while(!stopped){
            stopped = executionThread.getState() == State.TERMINATED;
        }
        //stops the drivetrain motors
        drivetrain.stop();

        splineLEDs.cancel();
    }

}

//class that is run by the thread to run the 
class ThreadedSplineExecutor extends Thread {
    private Command command;
    private volatile boolean done = false;
    private volatile Drivetrain drivetrain;
    private long period;

    /**
     * Creates a spline executor on its own thread to speed up period
     * @param command the spline to run
     * @param period the period in nanoseconds
     */
    ThreadedSplineExecutor(Command command, Drivetrain drivetrain, long period) {
        super();
        this.drivetrain = drivetrain;
        this.command = command;
        this.done = false;
        this.period = period;
    }

    @Override
    //gets called when thread starts
    public void run() {
        super.run();

        while (!command.isFinished() && !done) {

            long commandStartingTime = System.nanoTime();

            // Executes the ramsete command to set drivetrain motor powers
            drivetrain.updateOdometry();
            //put a measurement here
            command.execute();

            //System.out.println("command executed!");

            try {
                //sleep period ms to make the timing consistant
                long executeTime = (System.nanoTime() - commandStartingTime);
                long sleepPeriod = period - executeTime;

                if (sleepPeriod < 0) {
                    System.out.println(String.format("Loop overran with time %d", Math.abs((sleepPeriod)/1_000_00)));

                } else {
                    sleep(sleepPeriod / 1_000_000, (int) (sleepPeriod % 1_000_000));
                }

            } catch (InterruptedException e) {
                // chronic
                e.printStackTrace();
            }
        }
    }

    // Stops the thread if called for by another
    public synchronized void stopRunning(){
        synchronized(command){
            command.cancel();
        }
        done = true;
    }
}

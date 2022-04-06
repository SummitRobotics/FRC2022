// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.CommandThreader;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import java.util.List;
import java.util.function.Supplier;

/**
 * Command to follow a trajectory.
 * This command is threaded using Command Threader
 * This means the trajectory calculations runs every ~5ms
 */
public class FollowDynamicTrajectoryThreaded extends CommandBase {

    private final Drivetrain drivetrain;
    private CommandThreader commandThreader;
    private final int period;

    private final Supplier<Trajectory> generateTrajectory;

    private final LEDCall splineLEDs = new LEDCall(LEDPriorities.SPLINES, LEDRange.All)
        .sine(Colors.PURPLE);

    /**
     * Command to follow a trajectory object with a dynamic starting, midpoint and end pos.
     * The trajectory has been saved to the roborio with threading to make
     * it more precise
     *
     * @param drivetrain drivetrain to control
     * @param startingPos A supplier for the starting pos of the robot
     * @param midpoints A supplier for the midpoints for the trajectory
     * @param endingPos A supplier for the ending pos of the robot
     * @param config The trajectory config
     */
    public FollowDynamicTrajectoryThreaded(
        Supplier<Pose2d> startingPos,
        Supplier<Pose2d> endingPos,
        Supplier<List<Translation2d>> midpoints,
        TrajectoryConfig config,
        Drivetrain drivetrain
    ) {
        super();

        this.drivetrain = drivetrain;
        this.period = 5;

        generateTrajectory = () ->
            TrajectoryGenerator.generateTrajectory(startingPos.get(), midpoints.get(), endingPos.get(), config);

        addRequirements(drivetrain);
    }

    /**
     * Command to follow a trajectory object with dynamic waypoints.
     * The trajectory has been saved to the roborio with threading to make
     * it more precise.
     *
     * @param waypoints A supplier of the waypoints for the trajectory
     * @param config The trajectory config
     * @param drivetrain The drivetrain subsystem
     */

    public FollowDynamicTrajectoryThreaded(
        Supplier<List<Pose2d>> waypoints,
        TrajectoryConfig config,
        Drivetrain drivetrain
    ) {
        super();

        this.drivetrain = drivetrain;
        this.period = 5;

        generateTrajectory = () -> TrajectoryGenerator.generateTrajectory(waypoints.get(), config);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        splineLEDs.activate();
        Trajectory trajectory = generateTrajectory.get();

        drivetrain.getFieldWidget().getObject("trajectory").setTrajectory(trajectory);

        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                // TODO tune controller values
                new RamseteController(1.5, 0.8),
                Drivetrain.DriveKinimatics,
                drivetrain::setMotorTargetSpeed,
                drivetrain);

        //drivetrain.setPose(trajectory.getInitialPose());

        // Wraps the command, so we can update odometry every cycle.
        Runnable onExecute =
            () -> {
                drivetrain.updateOdometry();
                ramseteCommand.execute();
            };

        // Wraps the ramseteCommand in a functional command,
        // so we can update drivetrain odometry still.
        FunctionalCommand command = new FunctionalCommand(
            ramseteCommand::initialize,
            onExecute,
            ramseteCommand::end,
            ramseteCommand::isFinished,
            drivetrain);

        // Creates the command threader
        commandThreader = new CommandThreader(command, period, 10);
        commandThreader.initialize();

        System.out.println("thread started");
    }

    @Override
    public boolean isFinished() {
        // checks if thread is running or ended
        return commandThreader.isFinished();
    }

    @Override
    public void end(boolean interrupted) {

        // stops the thread
        commandThreader.end(interrupted);

        // stops the drivetrain motors
        drivetrain.stop();

        splineLEDs.cancel();
    }
}

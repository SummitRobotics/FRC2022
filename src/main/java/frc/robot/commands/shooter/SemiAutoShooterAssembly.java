package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.AxisPriorities;


/**
 * Command for running the shooter in full auto mode.
 */
public class SemiAutoShooterAssembly extends CommandBase {

    // subsystems
    private Shooter shooter;
    private Conveyor conveyor;

    // OI
    OIButton shootButton;
    OIButton.PrioritizedButton prioritizedShootButton;

    // tracker variables
    private ConveyorState indexState;
    private boolean isBallIndexed;
    
    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param shootButton The button to enable the shooter.
     */
    public SemiAutoShooterAssembly(Shooter shooter, Conveyor conveyor, OIButton shootButton) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shootButton = shootButton;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        shooter.stop();

        prioritizedShootButton = shootButton.prioritize(AxisPriorities.DEFAULT);
        indexState = conveyor.getWillBeIndexedState();
    }

    @Override
    public void execute() {
        // Tracker variables to prevent measurements from changing mid-cycle
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();


        if (prioritizedShootButton.get()
            && indexState != ConveyorState.NONE
            && isBallIndexed) {
            // Insert remaining shooter logic here.
        }
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedShootButton = null;
        prioritizedShootButton.destroy();
    }
}

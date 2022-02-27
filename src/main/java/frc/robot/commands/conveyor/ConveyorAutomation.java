package frc.robot.commands.conveyor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Command to automatically control the conveyor based on sensor measurements.
 */
public class ConveyorAutomation extends CommandBase {
    Conveyor conveyor;
    public ConveyorAutomation(Conveyor conveyor){
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }
    public void initialize(){
        conveyor.stop();
    }
    public void execute(){
        if (conveyor.getDoesBallExist() && !conveyor.getIsBallIndexed()){
            conveyor.setBeltMotorPower(1);
        }else{
            conveyor.setBeltMotorPower(0);
        }
    }
    @Override
    public void end(final boolean interrupted) {
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return !conveyor.getDoesBallExist();
    }
}

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
/**
 * Command to automatically control the conveyor based on sensor measurements.
 */
public class ConveyorAutomation extends CommandBase {
    private Conveyor conveyor;
    public ConveyorAutomation(Conveyor conveyor){
        this.conveyor = conveyor;
    }
    @Override
    public void initialize(){

    }
    
    @Override
    public void execute(){
        
        if (!conveyor.getIsBallIndexed() && conveyor.getDoesBallExist() && conveyor.getBeltEncoderPosition() == 0){
            conveyor.setBeltMotorPower(.5);
        }else if (!conveyor.getIsBallIndexed() && conveyor.getDoesBallExist() && conveyor.getBeltEncoderPosition() != 0 && conveyor.getBeltRPM() == 0){
            conveyor.setBeltMotorPower(.5);
        }
        if (conveyor.getIsBallIndexed()){
            conveyor.setBeltMotorPower(0);
            conveyor.setBeltEncoder(0);
        }
    }
    @Override
    public void end(final boolean interrupted) {
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

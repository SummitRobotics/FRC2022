package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import java.lang.Math;
import java.lang.reflect.Array;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    private DriverStation driverStation;
    private Shooter shooter;
    private Conveyor conveyor;
    private boolean teamClrIsBlue;
    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     */
    public FullAutoShooterAssembly(Shooter shooter, Conveyor conveyor, DriverStation driverStation) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.driverStation = driverStation;
        
        if (driverStation.getAlliance().toString() == "kRed"){
            teamClrIsBlue = false;
        }else{
            teamClrIsBlue = true;
        }
        this.teamClrIsBlue = teamClrIsBlue;
        addRequirements(shooter, conveyor);
        

    }

    public double distanceFinder(double tx){
        double startingAngle = 40;
        startingAngle += tx;
        double distance = Math.tan(startingAngle);
        distance = 2.64 / distance;

        return distance;
    }
    @Override
    public void execute() {
        double distance;
        NetworkTable limTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTable hoodTable = NetworkTableInstance.getDefault().getTable("Hood");
        NetworkTable speedTable = NetworkTableInstance.getDefault().getTable("RPM");
        NetworkTableEntry tx = limTable.getEntry("tx");
        distance = tx.getDouble(0);
        distance = distanceFinder(distance);
        distance = Math.round(distance);
        NetworkTableEntry Hood = hoodTable.getEntry(Double.toString(distance));
        NetworkTableEntry speed = speedTable.getEntry(Double.toString(distance));
        boolean HoodAngle = Hood.getBoolean(false);
        double motorPower = speed.getDouble(0);
        shooter.setHoodPos(HoodAngle);
        shooter.setMotorPower(motorPower);
        
        

    
    }
}

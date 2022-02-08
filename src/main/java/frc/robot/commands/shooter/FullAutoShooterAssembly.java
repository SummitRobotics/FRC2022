package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import java.lang.Math;
import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {

    // subsystems
    private Shooter shooter;
    private Conveyor conveyor;

    // other variables
    private NetworkTable hoodTable;
    private NetworkTable speedTable;
    private NetworkTableEntry tx;
    private double distance;
    private NetworkTableEntry speed;
    private NetworkTableEntry hood;
    private boolean hoodAngle;
    private double motorPower;


    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     */
    public FullAutoShooterAssembly(Shooter shooter, Conveyor conveyor) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        addRequirements(shooter);
    }

    /**
     * Finds the distance between the limelight and the target.
     *
     * @param tx The angle, in degrees of the camera.
     * @return distance The distance between the limelight and the target, in feet.
     */
    public double distanceFinder(double tx) {
        double startingAngle = 40;
        startingAngle += tx;
        double distance = Math.tan(startingAngle);
        distance = 2.64 / distance;

        return distance;
    }

    @Override
    public void execute() {
        
        NetworkTable limTable = NetworkTableInstance.getDefault().getTable("limelight");
        hoodTable = NetworkTableInstance.getDefault().getTable("Hood");
        speedTable = NetworkTableInstance.getDefault().getTable("RPM");
        tx = limTable.getEntry("tx");
        distance = tx.getDouble(0);
        distance = distanceFinder(distance);
        distance = Math.round(distance);
        hood = hoodTable.getEntry(Double.toString(distance));
        speed = speedTable.getEntry(Double.toString(distance));
        hoodAngle = hood.getBoolean(false);
        motorPower = speed.getDouble(0);
        shooter.setHoodPos(hoodAngle);
        shooter.setMotorPower(motorPower);
        

    
    }
}

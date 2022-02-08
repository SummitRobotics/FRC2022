package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import java.lang.Math;
import java.lang.reflect.Array;
import java.util.ArrayList;



/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    private Shooter shooter;
    private Conveyor conveyor;
    private ConveyorState teamClrIsBlue;
    private Drivetrain drivetrain;
    // other variables
    private NetworkTable hoodTable;
    private NetworkTable speedTable;
    private NetworkTableEntry tx;
    private double distance;
    private NetworkTableEntry speed;
    private NetworkTableEntry hood;
    private boolean hoodAngle;
    private double motorPower;
    private double motorSpeed;
    private NetworkTable limTable;
    private ConveyorState indexState;
    private boolean isBallIndexed;

    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param drivetrain The drivetrain subsystem.
     * @param limelight The limelight device.
     */
    public FullAutoShooterAssembly(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    // constants
    private static final double TARGET_MOTOR_SPEED = 0;

    // devices
    private Lemonlight limelight = new Lemonlight("Shooter");

    /**
     * Finds the distance between the limelight and the target.
     */
    @Override
    public void initialize() {
        hoodTable = NetworkTableInstance.getDefault().getTable("Hood");
        speedTable = NetworkTableInstance.getDefault().getTable("RPM");
        shooter.stop();
        if (DriverStation.getAlliance().toString() == "kRed") {
            teamClrIsBlue = ConveyorState.RED;
        } else {
            teamClrIsBlue = ConveyorState.BLUE;
        }
        
    }

    @Override
    public void execute() {        
        motorSpeed = shooter.getShooterVelocity();
        distance = limelight.getLimelightDistanceEstimateIN();
        distance = Math.round(distance);
        hood = hoodTable.getEntry(Double.toString(distance));
        speed = speedTable.getEntry(Double.toString(distance));
        hoodAngle = hood.getBoolean(false);
        motorPower = speed.getDouble(0);
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();
        if (isBallIndexed && indexState == teamClrIsBlue) { /* */ }
        if (motorSpeed != motorPower) {
            shooter.setMotorTargetSpeed(motorPower);
        }
        if (hoodAngle != shooter.getHoodPos()) {
            shooter.setHoodPos(hoodAngle);
        }
        
        
        

    
    }
}

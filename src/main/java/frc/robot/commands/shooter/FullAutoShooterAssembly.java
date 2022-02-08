package frc.robot.commands.shooter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.devices.Lemonlight;
import java.lang.Math;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Conveyor.ConveyorState;



/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    // Variables for various things, names are pretty explanitory
    private Shooter shooter;
    private Conveyor conveyor;
    private ConveyorState teamClrIsBlue;
    private Drivetrain drivetrain;
    private NetworkTable hoodTable;
    private NetworkTable speedTable;
    private double distance;
    private NetworkTableEntry speed;
    private NetworkTableEntry hood;
    private boolean hoodAngle;
    private double motorPower;
    private double motorSpeed;
    private ConveyorState indexState;
    private boolean isBallIndexed;
    private ShooterStates motorState;
    private ShooterStates hoodState;

    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     */
    public FullAutoShooterAssembly(Shooter shooter, Conveyor conveyor, Drivetrain drivetrain) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.drivetrain = drivetrain;
        addRequirements(shooter);
    }
    // Enum for setting state
    public enum ShooterStates{
        IDLE,
        REVVING,
        READY,
        MISALIGNED, 
        UNKNOWN
    }

    // devices
    private Lemonlight limelight = new Lemonlight("Shooter");

    /**
     * Initializing variables
     */
    @Override
    public void initialize(){
        hoodTable = NetworkTableInstance.getDefault().getTable("Hood");
        speedTable = NetworkTableInstance.getDefault().getTable("RPM");
        shooter.stop();
        if (DriverStation.getAlliance().toString() == "kRed"){
            teamClrIsBlue = ConveyorState.RED;
        }else{
            teamClrIsBlue = ConveyorState.BLUE;
        }
        motorState = ShooterStates.IDLE;
        hoodState = ShooterStates.UNKNOWN;
    }
    @Override
    public void execute() { 
        // Checking Variable       
        motorSpeed = shooter.getShooterVelocity();
        distance = limelight.getLimelightDistanceEstimateIN();
        distance = Math.round(distance);
        hood = hoodTable.getEntry(Double.toString(distance));
        speed = speedTable.getEntry(Double.toString(distance));
        hoodAngle = hood.getBoolean(false);
        motorPower = speed.getDouble(0);
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();
        //Checking to make sure we have a ball and it is the same color as our team
        if (isBallIndexed && indexState == teamClrIsBlue){
            if (motorSpeed != motorPower){
                shooter.setMotorTargetSpeed(motorPower);
                motorState = ShooterStates.REVVING;
            } else{
                motorState = ShooterStates.READY;
            }
            if (hoodAngle != shooter.getHoodPos()) {
                shooter.setHoodPos(hoodAngle);
                hoodState = ShooterStates.MISALIGNED;
            }else{
                hoodState = ShooterStates.READY;
            }
            if (motorState == ShooterStates.READY && hoodState == ShooterStates.READY){
                //TODO Add Code to actually shoot, check offset
            }

        }
        
        
        

    
    }
}

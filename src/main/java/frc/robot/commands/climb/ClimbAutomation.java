// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.PIDValues;
import frc.robot.utilities.lists.Ports;



/**
 * Climb automation command.
 */
public class ClimbAutomation extends CommandBase {
    // Adding network tables for vision
    NetworkTableEntry angleToTurn;
    NetworkTableEntry distToMove;
    // Pid Variables
    protected static final double 
            ALIGN_P = PIDValues.ALIGN_P,
            ALIGN_I = PIDValues.ALIGN_I,
            ALIGN_D = PIDValues.ALIGN_D,
            MOVE_P = PIDValues.MOVE_P,
            MOVE_I = PIDValues.MOVE_I,
            MOVE_D = PIDValues.MOVE_D,
            CLIMB_P = PIDValues.CLIMB_P,
            CLIMB_I = PIDValues.CLIMB_I,
            CLIMB_D = PIDValues.CLIMB_D,
            HEIGHT_OF_BAR = 3;
    protected PIDController movePID;
    protected PIDController alignPID;
    // Initializing Subsystems
    Climb climb;
    Drivetrain drivetrain;
    Ports ports;
    RollingAverage avgLeftScrewPower;
    RollingAverage avgLeftMotorPower;
    RollingAverage avgRightScrewPower;
    RollingAverage avgRightMotorPower;
    private int loopCount;
    private int attachLoop;
    private int extendLoop;
    /**
     * Climb states.
     */
    
    public enum ClimbStates {
        EXTENDED,
        RETRACTED,
        LATCHED,
        DONE,
        WORKING,
        BROKEN

    }

    /**
     * Motor states.
     */
    public enum MotorStates {
        EXTENDING,
        RETRACTING,
        IDLE,
        TESTING,
        BROKEN
    }

    /**
     * Piston states.
     */
    public enum PistonStates {
        EXTENDED,
        RETRACTED
    }

    // Creating tracking processes
    // motors
    MotorStates motorLeft;
    MotorStates motorRight;
    // pivot pistons
    PistonStates pivotPistLeft;
    PistonStates pivotPistRight;
    // clamp pistons
    PistonStates clampPistLeft;
    PistonStates clampPistRight;
    // tracking the system as a whole (makes it easier to check in if statements :))
    ClimbStates climbSystem;
    // various variables
    int barNumber;
    int normalDrivePower;
    int normalScrewPower;
    int normalTestDrivePower;
    int normalTestScrewPower;
    boolean hasHorizonalDistance;
    double turnAngle;
    double moveDist;
    String barMisaligned; 
    boolean stateChanged;

    /** Creates a new ClimbAutomation. 
     *
     * @param climb climb subsystem
     * @param drivetrain Drivetrain subsystem
    */
    public ClimbAutomation(Climb climb, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.climb = climb;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDs.getInstance().addCall("Climbing", new LEDCall(LEDPriorities.CLIMBING, LEDRange.All));
        barMisaligned = "";
        climb.stop();
        this.alignPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        this.movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);
        motorLeft = MotorStates.IDLE;
        motorRight = MotorStates.IDLE;
        climb.setPivotPos(false);
        climb.setLeftDetachPos(true);
        climb.setRightDetachPos(true);
        pivotPistLeft = PistonStates.RETRACTED;
        pivotPistRight = PistonStates.RETRACTED;
        clampPistLeft = PistonStates.RETRACTED;
        clampPistRight = PistonStates.RETRACTED;
        climbSystem = ClimbStates.DONE;
        barNumber = 0;
        loopCount = 0;
        attachLoop = 0;
        extendLoop = 0;
        hasHorizonalDistance = false;
        motorLeft = MotorStates.IDLE;
        motorRight = MotorStates.IDLE;
        // get the default instance of NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // get a reference to the subtable called "datatable"
        //NetworkTable datatable = inst.getTable("Vision");

        // get a reference to key in "datatable" called "Y"
        // angleToTurn = datatable.getEntry("Target Angle");
        // distToMove = datatable.getEntry("Target Angle");
        inst.startClientTeam(5468);

    }

    // Extending to grab bar using screws
    private void extend() {
        climb.setMotorPosition(climb.BACK_LIMIT);
        System.out.println("tryingToEnxtendDDDDDDDDDDDDDDDDDDDDDD");
        System.out.println(climb.getRightEncoderValue());
        System.out.println(extendLoop);
        if (climb.getLeftEncoderValue() <= climb.BACK_LIMIT + 1 && climb.getRightEncoderValue() <= climb.BACK_LIMIT + 1) {
            if (barNumber == 0) {
                climbSystem = ClimbStates.EXTENDED;
            } else if (extendLoop > 50) {
                climbSystem = ClimbStates.EXTENDED;
                loopCount = 0;
                attachLoop = 0;
            } else {
                climb.setPivotPos(false);
                extendLoop++;
            }
            System.out.println("ext2");
        }
    }

    // retracting screws
    private void retract() {
        climb.setPivotPos(false);
        climb.setMotorPosition(climb.FORWARD_LIMIT);
        if (climb.isHooked() || barNumber == 0){
            climb.setLeftDetachPos(true);
            climb.setRightDetachPos(true);
        } 
        if (climb.getLeftEncoderValue() >= climb.FORWARD_LIMIT - .2 && climb.getRightEncoderValue() >= climb.FORWARD_LIMIT - .2) {
            System.out.println("WITHIN LIMIT");
            climb.setRightDetachPos(false);
            climb.setLeftDetachPos(false);
            if (!climb.getLeftDetachPos() && !climb.getRightDetachPos()){
                if (attachLoop >= 10){
                    climbSystem = ClimbStates.LATCHED;
                    System.out.println("LATCHEEEEEEEEEEEEEEEEEEEEEEEEEED");
                } else {
                    attachLoop++;
                    System.out.println("incrementing");
                }
            }
        }
    }

    // cycling bot, making sure that latches are on by using power detection
    private void cycle() {
        climb.setPivotPos(true);
        climb.setMotorPosition(-10);
    
        if (climb.getLeftEncoderValue() <= -9 && climb.getRightEncoderValue() <= -9) {
            climbSystem = ClimbStates.DONE;
            stateChanged = true;
            barNumber++;
            extendLoop = 0;
            System.out.println(barNumber);
        }

    }
    // set camera height in pixels
    // update with network tables code

    private boolean aligned() {
        double angle = angleToTurn.getDouble(0.0);
        double dist = distToMove.getDouble(0.0);
        double alignPower = alignPID.calculate(angle);
        double leftPower = 0;
        double rightPower = 0;
        leftPower += alignPower;
        rightPower += -alignPower;
        double offsetDistance = Math.tan(Math.toRadians(dist)) * HEIGHT_OF_BAR;
        if (!Functions.isWithin(1, offsetDistance, .5)) {
            // records current distance to be heald with pid, clamped to the max and min
            // range of the shooter
            if (!hasHorizonalDistance) {
                movePID.setSetpoint(Functions.clampDouble(offsetDistance, 1, 1.5));
                hasHorizonalDistance = true;
            }

            double movePower = movePID.calculate(offsetDistance);

            leftPower += movePower;
            rightPower += movePower;

        }

        // sets drivetrain powers
        drivetrain.setLeftMotorPower(leftPower);
        drivetrain.setRightMotorPower(rightPower);

        return alignPID.atSetpoint() && movePID.atSetpoint();
    }
    /**
     * aligns climb using limit switches.
     *
     * @return aligned
     */

    public boolean alignByLimit() {
        if (loopCount < 20) {
            if (climb.getLeftLimit() && barMisaligned == "" && !climb.getRightLimit()) {
                barMisaligned = "left";
                drivetrain.setLeftMotorPower(0);
                return false;
            } else if (climb.getRightLimit() && barMisaligned == "" && !climb.getLeftLimit()) {
                barMisaligned = "right";
                drivetrain.setRightMotorPower(0);
                return false;
            } else if (!climb.getLeftLimit() && !climb.getRightLimit()) {
                drivetrain.setBothMotorPower(.2);
                barMisaligned = "";
                return false;
            } else {
                loopCount += 1;
                if (loopCount >= 20) {
                    drivetrain.setBothMotorPower(0);
                    return true;

                } else {
                    drivetrain.setBothMotorPower(.2);
                    return false;
                }
            }

        } else {

            return true;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (barNumber < 3) {
            if (barNumber == 0) {
                // get rid of aligned() here if using alignByLimit()
                if (climbSystem == ClimbStates.DONE) {
                    extend();
                    System.out.println("extend 1");
                    
                //replace touchingBar() with alignByLimit() if using it
                } else if (climbSystem == ClimbStates.EXTENDED && alignByLimit()) {
                    retract();
                    System.out.println("retract 1");
                } else if (climbSystem == ClimbStates.LATCHED) {
                    cycle();
                    System.out.println("cycle 1");
                } else if (climbSystem == ClimbStates.BROKEN) {
                    climb.stop();
                    System.out.println("brk 1");
                }
            } else if (barNumber <= 2) {
                if (climbSystem == ClimbStates.DONE) {
                    extend();
                    System.out.println("extending 2");
                } else if (climbSystem == ClimbStates.EXTENDED) {
                    retract();
                    System.out.println("retracting 2");
                } else if (climbSystem == ClimbStates.LATCHED) {
                    if (barNumber != 2) {
                        cycle();
                        System.out.println("cycle 2");
                    }
                } else if (climbSystem == ClimbStates.BROKEN) {
                    climb.stop();
                    System.out.println("brk 1");
                }
            }
        } else {
            LEDs.getInstance().removeCall("Climbing");
            climb.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LEDs.getInstance().removeCall("Climbing");
        climb.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

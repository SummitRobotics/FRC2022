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
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.AxisPriorities;
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
    // PID controllers
    protected PIDController movePID;
    protected PIDController alignPID;
    protected PIDController climbLeftPID;
    protected PIDController climbRightPID;
    // Initializing Subsystems
    Climb climb;
    Drivetrain drivetrain;
    Ports ports;
    RollingAverage avgLeftScrewPower;
    RollingAverage avgLeftMotorPower;
    RollingAverage avgRightScrewPower;
    RollingAverage avgRightMotorPower;

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
    private final LEDCall climbingLedCall = new LEDCall(LEDPriorities.ARMS_UP, LEDRange.All).sine(Colors.ORANGE);
    // OI
    OIButton climbButton;
    OIButton.PrioritizedButton prioritizedClimbButton;

    /** Creates a new ClimbAutomation. 
     *
     * @param climb climb subsystem
     * @param drivetrain Drivetrain subsystem
     * @param climbButton climb button.
    */
    public ClimbAutomation(Climb climb, Drivetrain drivetrain, OIButton climbButton) {
        this.climbButton = climbButton;
        this.drivetrain = drivetrain;
        this.climb = climb;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        barMisaligned = "";
        this.alignPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        this.movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);
        this.climbLeftPID = new PIDController(CLIMB_P, CLIMB_I, CLIMB_D);
        this.climbRightPID = new PIDController(CLIMB_P, CLIMB_I, CLIMB_D);
        climb.stop();
        prioritizedClimbButton = climbButton.prioritize(AxisPriorities.DEFAULT);
        motorLeft = MotorStates.IDLE;
        motorRight = MotorStates.IDLE;
        climb.setPivotPos(false);
        climb.setLeftDetachPos(false);
        climb.setRightDetachPos(false);
        climb.zeroEncoders();
        pivotPistLeft = PistonStates.RETRACTED;
        pivotPistRight = PistonStates.RETRACTED;
        clampPistLeft = PistonStates.RETRACTED;
        clampPistRight = PistonStates.RETRACTED;
        climbSystem = ClimbStates.DONE;
        barNumber = 0;
        avgLeftMotorPower = new RollingAverage(5, false);
        avgRightScrewPower = new RollingAverage(5, false);
        avgRightMotorPower = new RollingAverage(5, false);
        avgLeftScrewPower = new RollingAverage(5, false);
        // TODO find actual value in testing
        normalDrivePower = 0;
        normalTestDrivePower = 0;
        normalScrewPower = 0;
        normalTestScrewPower = 0;
        hasHorizonalDistance = false;
        motorLeft = MotorStates.IDLE;
        motorRight = MotorStates.IDLE;
        // get the default instance of NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // get a reference to the subtable called "datatable"
        NetworkTable datatable = inst.getTable("Vision");

        // get a reference to key in "datatable" called "Y"
        angleToTurn = datatable.getEntry("Target Angle");
        distToMove = datatable.getEntry("Target Angle");
        inst.startClientTeam(5468);
        alignPID.setTolerance(1, 1);
        movePID.setTolerance(1, 1);
        alignPID.setSetpoint(0);
        movePID.setSetpoint(0);
        climbLeftPID.setSetpoint(0);
        climbLeftPID.setTolerance(1, 1);

    }

    // Extending to grab bar using screws
    private void extend() {
        if ((climb.getRightEncoderValue() < 65 || climb.getLeftEncoderValue() < 65) && climbSystem != ClimbStates.EXTENDED) {
            climbLeftPID.setSetpoint(66);
            climbRightPID.setSetpoint(66);
            climb.setLeftMotorPower(climbLeftPID.calculate(climb.getLeftEncoderValue()));
            climb.setRightMotorPower(climbRightPID.calculate(climb.getRightEncoderValue()));

        }
        if (climbLeftPID.atSetpoint() && climbRightPID.atSetpoint()) {
            climbSystem = ClimbStates.EXTENDED;

        }
    }

    // retracting screws
    private void retract() {
        if (climb.getRightEncoderValue() > 2 || climb.getLeftEncoderValue() > 2) {
            climbLeftPID.setSetpoint(0);
            climbRightPID.setSetpoint(0);
            climb.setLeftMotorPower(climbLeftPID.calculate(climb.getLeftEncoderValue()));
            climb.setRightMotorPower(climbRightPID.calculate(climb.getRightEncoderValue()));
        }
        if (climbLeftPID.atSetpoint() && climbRightPID.atSetpoint()) {
            climbSystem = ClimbStates.RETRACTED;

        }
    }

    // cycling bot, making sure that latches are on by using power detection
    private void cycle() {
        if (motorRight != MotorStates.EXTENDING) {
            climb.setLeftMotorVelocity(1);
            climb.setLeftMotorVelocity(1);
            motorRight = MotorStates.EXTENDING;
            motorLeft = MotorStates.EXTENDING;
        }
        if (climb.getRightEncoderValue() > 10 && climb.getLeftEncoderValue() > 10) {
            climb.setRightMotorVelocity(0);
            climb.setLeftMotorVelocity(0);
            climb.setPivotPos(true);
            climbSystem = ClimbStates.DONE;
            motorRight = MotorStates.IDLE;
            motorLeft = MotorStates.IDLE;
        }

    }
    // set camera height in pixels
    // update with network tables code

    private boolean touchingBar() {
        if (climb.getRightLimit() && climb.getLeftLimit()) {
            drivetrain.setBothMotorPower(0);
            return true;
        } else {
            drivetrain.setBothMotorPower(.1);
            return false;
        }
    }

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
     */

    public void alignByLimit() {
        if (!climb.getLeftLimit() && barMisaligned == "" && climb.getRightLimit()) {
            barMisaligned = "left";
            drivetrain.setLeftMotorPower(0);
        } else if (!climb.getRightLimit() && barMisaligned == "" && climb.getLeftLimit()) {
            barMisaligned = "right";
            drivetrain.setRightMotorPower(0);
        } else if (climb.getLeftLimit() && climb.getRightLimit()) {
            drivetrain.setBothMotorPower(-.5);
            barMisaligned = "";
        } else {
            drivetrain.setBothMotorPower(0);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climb.updateClimbAverage();
        if (prioritizedClimbButton.get() && barNumber < 3) {
            climbingLedCall.activate();
            if (barNumber == 0) {
                // get rid of aligned() here if using alignByLimit()
                if (climbSystem == ClimbStates.DONE && aligned()) {
                    extend();
                //replace touchingBar() with alignByLimit() if using it
                } else if (climbSystem == ClimbStates.EXTENDED && touchingBar()) {
                    climbLeftPID.reset();
                    climbRightPID.reset();
                    retract();
                } else if (climbSystem == ClimbStates.LATCHED) {
                    cycle();
                } else if (climbSystem == ClimbStates.BROKEN) {
                    climb.stop();
                }
            } else if (barNumber <= 2) {
                if (climbSystem == ClimbStates.DONE) {
                    extend();
                } else if (climbSystem == ClimbStates.EXTENDED) {
                    climbLeftPID.reset();
                    climbRightPID.reset();
                    retract();
                } else if (climbSystem == ClimbStates.LATCHED) {
                    cycle();
                } else if (climbSystem == ClimbStates.BROKEN) {
                    climb.stop();
                }
            }
        } else {
            climbingLedCall.cancel();
            climb.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.stop();
        prioritizedClimbButton.destroy();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

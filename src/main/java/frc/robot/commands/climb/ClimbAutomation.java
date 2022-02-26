// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.climb;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.Ports;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.utilities.Functions;
import java.math.*;
public class ClimbAutomation extends CommandBase {
  // Adding network tables for vision
  NetworkTableEntry angleToTurn;
  NetworkTableEntry distToMove;
  //Pid Variables
  protected static final double
        ALIGN_P = 0,
        ALIGN_I = 0,
        ALIGN_D = 0,
        MOVE_P = 0,
        MOVE_I = 0,
        MOVE_D = 0,
        HEIGHT_OF_BAR = 3;
  // PID controllers
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
  // Creating Climber States
  public enum climbStates{
      EXTENDED,
      RETRACTED,
      LATCHED,
      DONE,
      WORKING,
      BROKEN
    
    }
  public enum motorStates{
    EXTENDING,
    RETRACTING,
    IDLE,
    TESTING,
    BROKEN
  }
  public enum pistonStates{
    EXTENDED,
    RETRACTED
  }
  // Creating tracking processes
    //motors
    motorStates motorLeft;
    motorStates motorRight;
    // pivot pistons
    pistonStates pivotPistLeft;
    pistonStates pivotPistRight;
    //clamp pistons
    pistonStates clampPistLeft;
    pistonStates clampPistRight;
    //tracking the system as a whole (makes it easier to check in if statements :))
    climbStates climbSystem;
    //various variables
    int barNumber;
    int normalDrivePower;
    int normalScrewPower;
    int normalTestDrivePower;
    int normalTestScrewPower;
    boolean hasHorizonalDistance;
    double turnAngle;
    double moveDist;
    private final LEDCall climbingLedCall =
    new LEDCall(LEDPriorities.ARMS_UP, LEDRange.All).sine(Colors.ORANGE);
  // OI
  OIButton climbButton;
  OIButton.PrioritizedButton prioritizedClimbButton;
  /** Creates a new ClimbAutomation. */
  public ClimbAutomation(Climb climb, Drivetrain drivetrain, OIButton climbButtonS) {
    this.climbButton = climbButton;
    this.drivetrain = drivetrain;
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.alignPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
    this.movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);
    climb.stop();
    prioritizedClimbButton =  climbButton.prioritize(AxisPriorities.DEFAULT);
    motorLeft = motorStates.IDLE;
    motorRight = motorStates.IDLE; 
    climb.setPivotPos(false);
    climb.setLeftDetachPos(false);
    climb.setRightDetachPos(false);
    climb.zeroEncoders();
    pivotPistLeft = pistonStates.RETRACTED; 
    pivotPistRight = pistonStates.RETRACTED; 
    clampPistLeft = pistonStates.RETRACTED; 
    clampPistRight = pistonStates.RETRACTED; 
    climbSystem = climbStates.DONE; 
    barNumber = 0;
    avgLeftMotorPower = new RollingAverage(5, false);
    avgRightScrewPower = new RollingAverage(5, false);
    avgRightMotorPower = new RollingAverage(5, false);
    avgLeftScrewPower = new RollingAverage(5, false);
    //TODO find actual value in testing
    normalDrivePower = 0;
    normalTestDrivePower = 0;
    normalScrewPower = 0;
    normalTestScrewPower = 0;
    hasHorizonalDistance = false;
    motorLeft = motorStates.IDLE;
    motorRight = motorStates.IDLE;
    //get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //get a reference to the subtable called "datatable"
    NetworkTable datatable = inst.getTable("Vision");

    //get a reference to key in "datatable" called "Y"
    angleToTurn = datatable.getEntry("Target Angle");
    distToMove = datatable.getEntry("Target Angle");
    inst.startClientTeam(5468);

  }
  // Extending to grab bar using screws
  public void extend(){
    if ((climb.getRightEncoderValue() < 66.66 || climb.getLeftEncoderValue() < 66.66)
       && motorRight == motorStates.IDLE && motorLeft == motorStates.IDLE){
      climb.setMotorPower(1);
      motorLeft = motorStates.EXTENDING;
      motorRight = motorStates.EXTENDING;
      
    }
    if(climb.getLeftEncoderValue() >= 66.66 && motorLeft == motorStates.EXTENDING){
      climb.setLeftMotorPower(0);
      motorLeft = motorStates.IDLE;
    }
    if(climb.getRightEncoderValue() >= 66.66 && motorRight == motorStates.EXTENDING){
      climb.setRightMotorPower(0);
      motorRight = motorStates.IDLE;
    if (motorRight == motorStates.IDLE && motorLeft == motorStates.IDLE){
      climbSystem = climbStates.EXTENDED;
      climb.setPivotPos(false);

    }

    }
  }
  //TODO add gyro code
  // retracting screws 
  public void retract(){
    if (climb.isHooked() && motorRight == motorStates.TESTING){
          climb.setMotorPower(-1);
          climb.setLeftDetachPos(false);
          climb.setRightDetachPos(false);
          motorLeft = motorStates.RETRACTING;
          motorRight = motorStates.RETRACTING;
    }else if(motorRight == motorStates.IDLE){
      climb.setMotorPower(-.1);
      motorLeft = motorStates.TESTING;
      motorRight = motorStates.TESTING;
    }else if(motorRight == motorStates.TESTING && climb.getRightEncoderValue() < 30){
      climbSystem = climbStates.BROKEN;
      motorRight = motorStates.BROKEN;
      motorLeft = motorStates.BROKEN;
      climb.stop();
    }
    
    if (climb.getRightEncoderValue() <= 0 && climb.getLeftEncoderValue() <= 0){
      climb.setMotorPower(0);
      climb.setRightDetachPos(true);
      climb.setLeftDetachPos(true);
      climbSystem = climbStates.LATCHED;
    }
  }
  // cycling bot, making sure that latches are on by using power detection
  public void cycle(){
    if (motorRight != motorStates.EXTENDING){
      climb.setRightMotorPower(1);
      climb.setLeftMotorPower(1);
      motorRight = motorStates.EXTENDING;
      motorLeft = motorStates.EXTENDING;
    }
    if (climb.getRightEncoderValue() > 10 && climb.getLeftEncoderValue() > 10){
      climb.setLeftMotorPower(0);
      climb.setRightMotorPower(0);
      climb.setPivotPos(true);
      climbSystem = climbStates.DONE;
      motorRight = motorStates.IDLE;
      motorLeft = motorStates.IDLE;
    }

  }
  // update with network tables code
  public boolean aligned(){
    double angle = angleToTurn.getDouble(0.0);
    double dist = distToMove.getDouble(0.0);
    double alignPower = alignPID.calculate(angle);
    double leftPower = 0;
    double rightPower = 0;
    leftPower += alignPower;
    rightPower += -alignPower;
    double offsetDistance = Math.tan(Math.toRadians(dist))*HEIGHT_OF_BAR;
    if (!Functions.isWithin(1, offsetDistance, .5)) {
      //records current distance to be heald with pid, clamped to the max and min range of the shooter
      if (!hasHorizonalDistance) {
          movePID.setSetpoint(Functions.clampDouble(offsetDistance, 1, 1.5));
          hasHorizonalDistance = true;
      }

      double movePower = movePID.calculate(offsetDistance);

      leftPower += movePower;
      rightPower += movePower;

    }
    
    //sets drivetrain powers
    drivetrain.setLeftMotorPower(leftPower);
    drivetrain.setRightMotorPower(rightPower);

    return alignPID.atSetpoint() && movePID.atSetpoint();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    avgLeftScrewPower.update(climb.getLeftCurrentDraw());
    avgRightScrewPower.update(climb.getRightCurrentDraw());
    climb.updateClimbAverage();
    if (prioritizedClimbButton.get() && barNumber < 3){
      climbingLedCall.activate();
      if (barNumber == 0){
        if (climbSystem == climbStates.DONE && aligned()){
          extend();
        }else if (climbSystem == climbStates.EXTENDED){
          retract();
        }else if (climbSystem == climbStates.LATCHED){
          cycle();
        }else if (climbSystem == climbStates.BROKEN){

        }
      }else if (barNumber <= 2){
          if (climbSystem == climbStates.DONE){
            extend();
          }else if (climbSystem == climbStates.EXTENDED){
            retract();
          }else if (climbSystem == climbStates.LATCHED){
            cycle();
          }else if (climbSystem == climbStates.BROKEN){

          }
      }
      
      
    }else{
      climbingLedCall.cancel();
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

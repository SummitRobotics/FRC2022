package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Homeable;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Ports;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Subsystem for the Climb Subsystem.
 */
public class Climb extends SubsystemBase implements Testable {

    private AHRS gyro;
    private final DigitalInput leftClimbLimit = new DigitalInput(Ports.LEFT_LIMIT_SWITCH);
    private final DigitalInput rightClimbLimit = new DigitalInput(Ports.RIGHT_LIMIT_SWITCH);
    RollingAverage climbPitchAverage = new RollingAverage(10, true);
    private double oldGyroAngle = 0;
    private RollingAverage climbDrivitiveAvrage = new RollingAverage(10, true);
    private boolean wereSwitchesFalse = false;

    // TODO set these
    public static final double
            P = 1,
            I = 0,
            D = 0,
            FF = 0,
            IZ = 0,
            // TODO tune these values
            CLIMB_TILT_ANGLE = -2,
            CLIMB_ROLL_ANGLE = 5,
            CLIMB_DRIVITIVE = 1,
            FORWARD_LIMIT = 0,
            BACK_LIMIT = -155,
            GRAB_POINT = -100;


    // Climb Motors
    private final CANSparkMax leftMotor =
            new CANSparkMax(Ports.LEFT_CLIMB_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor =
            new CANSparkMax(Ports.RIGHT_CLIMB_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    // Pivot Solenoid
    private final Solenoid pivot =
            new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.PIVOT_CLIMB_SOLENOID);

    // Detach Solenoids
    private final Solenoid leftDetach =
            new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.LEFT_DETACH_SOLENOID);
    private final Solenoid rightDetach =
            new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.RIGHT_DETACH_SOLENOID);

    // Solenoid Positions
    private boolean
            pivotPos = false,
            leftDetachPos = false,
            rightDetachPos = false;

    // PID Controllers
    private final SparkMaxPIDController leftPidController = leftMotor.getPIDController();
    private final SparkMaxPIDController rightPidController = rightMotor.getPIDController();

    // Encoders
    private final RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

    /**
     * Public Constructor for climb subsystem.
     *
     * @param gyro
     * 
     */
    public Climb(AHRS gyro) {
        leftPidController.setP(P);
        leftPidController.setI(I);
        leftPidController.setD(D);
        leftPidController.setFF(FF);
        leftPidController.setIZone(IZ);
        leftPidController.setOutputRange(-1.0, 1.0);

        rightPidController.setP(P);
        rightPidController.setI(I);
        rightPidController.setD(D);
        rightPidController.setFF(FF);
        rightPidController.setIZone(IZ);
        rightPidController.setOutputRange(-1.0, 1.0);
        this.gyro = gyro;
        //zeroEncoders();
   

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Zeros the climb motor encoders.
     */
    public void zeroEncoders() {
        leftMotorEncoder.setPosition(0);
        rightMotorEncoder.setPosition(0);
    }

    /**
     * Gets the left climb encoder value.
     *
     * @return The number of rotations the left encoder has gone through
     *      since its last reset.
     */
    public double getLeftEncoderValue() {
        return leftMotorEncoder.getPosition();
    }

    /**
     * Gets the right climb encoder value.
     *
     * @return The number of rotations the right encoder has gone through
     *      since its last reset.
     */
    public double getRightEncoderValue() {
        return rightMotorEncoder.getPosition();
    }

    /**
     * Gets the RPM of the left climb motor.
     *
     * @return The current velocity of the left climb motor in RPM.
     */
    public double getLeftMotorVelocity() {
        return leftMotorEncoder.getVelocity();
    }

    /**
     * Gets the RPM of the right climb motor.
     *
     * @return The current velocity of the right climb motor in RPM.
     */
    public double getRightMotorVelocity() {
        return rightMotorEncoder.getVelocity();
    }

    /**
     * Gets the current position of the pneumatic pivot.
     *
     * @return The current position of the pivot piston.
     */
    public boolean getPivotPos() {
        return pivotPos;
    }

    public void setLeftMotorVelocity(double speed) {
        leftMotor.set(speed);
    }

    public void setRightMotorVelocity(double speed) {

        leftMotor.set(speed);
    }

    public void setBothMotorVelocity(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    /**
     * Gets the current position of the pneumatic detach for the left side.
     *
     * @return The current position of the left side detach piston.
     */
    public boolean getLeftDetachPos() {
        return leftDetachPos;
    }

    /**
     * Gets the current position of the pneumatic detach for the right side.
     *
     * @return Teh current position of the right side detach piston.
     */
    public boolean getRightDetachPos() {
        return rightDetachPos;
    }

    /**
     * Sets the left motor power.
     *
     * @param power The power to set the left motor.
     */
    public void setLeftMotorPower(double power) {
        leftMotor.set(power);
    }

    /**
     * Sets the right motor power.
     *
     * @param power The power to set the right motor.
     */
    public void setRightMotorPower(double power) {
        rightMotor.set(power);
    }

    /**
     * Sets the power for both of the climb motors.
     *
     * @param power The power to set both the climb motors.
     */
    public void setMotorPower(double power) {
        setLeftMotorPower(power);
        setRightMotorPower(power);
    }

    /**
     * Sets the position to move the right motor to.
     * This uses PID and requires accurate PID values.
     *
     * @param position The position to set the motor to. (Revolutions)
     */
    public void setRightMotorPosition(double position) {
        rightPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the position to move the left motor to.
     * This uses PID and requires accurate PID values.
     *
     * @param position The position to set the motor to. (Revolutions)
     */
    public void setLeftMotorPosition(double position) {
        leftPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the position to move both the motors to.
     * This uses PID and requires accurate PID values.
     *
     * @param position The position to set the motor to. (Revolutions)
     */
    public void setMotorPosition(double position) {
        setRightMotorPosition(position);
        setLeftMotorPosition(position);
    }
    
    /**
     * Sets the pivot position for the pneumatics.
     *
     * @param pos The position to set it to (Boolean)
     */
    public void setPivotPos(boolean pos) {
        pivotPos = pos;
        pivot.set(pos);
    }

    /**
     * Toggles the position of the pivot.
     */
    public void togglePivotPos() {
        setPivotPos(!pivotPos);
    }

    /**
     * Sets the position for the left detach pneumatics.
     *
     * @param pos The position to set the left detach pneumatics.
     */
    public void setLeftDetachPos(boolean pos) {
        leftDetachPos = pos;
        leftDetach.set(pos);
    }

    /**
     * Checks to see if both arms are hooked, not sure if needed ¯\_(ツ)_/¯.
     *
     * @return rollIsLevel
     * 
     */

    public boolean isRollLevel() {
        return (gyro.getYaw() < CLIMB_ROLL_ANGLE);
    }

    /**
     * checks to see if the robot is hooked.
     *
     * @return isHooked
     */

    public boolean isHooked() {
        //System.out.println(climbPitchAverage.getAverage());
        return (climbPitchAverage.getAverage() < CLIMB_TILT_ANGLE); // && !isSwinging();
    }

    public boolean isSwinging() {
        System.out.println("driv: " + climbDrivitiveAvrage.getAverage());
        return climbDrivitiveAvrage.getAverage() < CLIMB_DRIVITIVE;
    }

    /**
     * checks if touching limit switch.
     *
     * @return is left climb touching limit switch
     */

    public boolean getLeftLimit() {
        return leftClimbLimit.get();
    }

    /**
     * checks if touching limit switch.
     *
     * @return is right climb touching limit switch
     */

    public boolean getRightLimit() {
        return rightClimbLimit.get();
    }

    /**
     * Toggles the position of the left detach piston.
     */

    public void toggleLeftDetachPos() {
        setLeftDetachPos(!leftDetachPos);
    }

    /**
     * Sets the position for the right detach pneumatics.
     *
     * @param pos The position to set the right detach pneumatics.
     */
    public void setRightDetachPos(boolean pos) {
        rightDetachPos = pos;
        rightDetach.set(pos);
    }

    /**
     * Toggles the position of the right detach piston.
     */
    public void toggleRightDetachPos() {
        setRightDetachPos(!rightDetachPos);
    }

    /**
     * Sets the position for both the right and left detach pneumatics.
     *
     * @param pos The position to set the pistons to.
     */
    public void setDetachPos(boolean pos) {
        setRightDetachPos(pos);
        setLeftDetachPos(pos);
    }

    /**
     * Stops the motors.
     */

    public void stop() {
        setLeftMotorPower(0);
        setRightMotorPower(0);
    }

    public double[] getPID() {
        return new double[] {P, I, D, FF, IZ};
    }

    @Override
    public void periodic() {
        double pa = gyro.getRoll();
        //System.out.println(pa);
        //updates drivitive calculation
        climbPitchAverage.update(pa);
        climbDrivitiveAvrage.update(Math.abs(pa - oldGyroAngle));
        oldGyroAngle = pa;
    }

    @Override
    public String getTestName() {
        return "Climb";
    }

    @Override
    public ArrayList<CANSparkMax> getMotors() {
        ArrayList<CANSparkMax> result = new ArrayList<CANSparkMax>();
        result.add(leftMotor);
        result.add(rightMotor);
        return result;
    }

    @Override
    public HashMap<String, Boolean> initCustomTests() {
        HashMap<String, Boolean> result = new HashMap<String, Boolean>();
        result.put("Left Limit Switch", false);
        result.put("Right Limit Swtich", false);

        return result;
    }

    @Override
    public HashMap<String, Boolean> runCustomTests() {
        HashMap<String, Boolean> result = new HashMap<String, Boolean>();

        if (!pivotPos && !wereSwitchesFalse) {
            setPivotPos(true);
        } else if (pivotPos && !getLeftLimit() && !getRightLimit()) {
            wereSwitchesFalse = true;
            setPivotPos(false);
        }

        if (wereSwitchesFalse) {
            result.put("Left Limit Switch", getLeftDetachPos());
            result.put("Right Limit Switch",  getRightDetachPos());
        } else {
            result.put("Left Limit Switch", false);
            result.put("Right Limit Switch",  false);
        }

        return result;
    }

    /**
     * Function to init telemetry for the climb subsystem.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("Climb");

        builder.addDoubleProperty("leftEncoderPosition", this::getLeftEncoderValue, null);
        builder.addDoubleProperty("rightEncoderPosition", this::getRightEncoderValue, null);
        // builder.addDoubleProperty("leftMotorVelocity", this::getLeftMotorVelocity, null);
        // builder.addDoubleProperty("rightMotorVelocity", this::getRightMotorVelocity, null);
        builder.addBooleanProperty("leftLimitSwitch", this::getLeftLimit, null);
        builder.addBooleanProperty("rightLimitSwitch", this::getRightLimit, null);
        // builder.addBooleanProperty("pivotPosition", this::getPivotPos, null);
        // builder.addBooleanProperty("leftDetachPosition", this::getLeftDetachPos, null);
        // builder.addBooleanProperty("rightDetachPosition", this::getRightDetachPos, null);
    }

    /**
     * Returns the homeable object for the left pivoting climb arm.
     *
     * @return the homeable object for the left pivoting climb arm
     */
    public Homeable getLeftArmHomeable() {
        return new Homeable() {

            @Override
            public double getCurrent() {
                return leftMotor.getOutputCurrent();
            }

            @Override
            public double getVelocity() {
                return leftMotorEncoder.getVelocity();
            }

            @Override
            public void setHomingPower(double power) {
                setLeftMotorPower(power);
            }

            @Override
            public void setHome(double position) {
                leftMotorEncoder.setPosition(position);
            }

            @Override
            public void setSoftLimits(double reverse, double forward) {
                leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) forward);
                leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverse);
            }

            @Override
            public void disableSoftLimits() {
                leftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
                leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
            }

            @Override
            public void enableSoftLimits() {
                leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
                leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            }

            @Override
            public Subsystem getSubsystemObject() {
                return new Subsystem() {
                    
                };
            }
        };
    }

    /**
     * Returns the homeable object for the right pivoting arm.
     *
     * @return the homeable object for the right pivoting arm
     */
    public Homeable getRightArmHomeable() {
        return new Homeable() {

            @Override
            public double getCurrent() {
                return rightMotor.getOutputCurrent();
            }

            @Override
            public double getVelocity() {
                return rightMotorEncoder.getVelocity();
            }

            @Override
            public void setHomingPower(double power) {
                setRightMotorPower(power);
            }

            @Override
            public void setHome(double position) {
                rightMotorEncoder.setPosition(position);
                
            }

            @Override
            public void setSoftLimits(double reverse, double forward) {
                rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float) forward);
                rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverse);
                
            }

            @Override
            public void disableSoftLimits() {
                rightMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
                rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
                
            }

            @Override
            public void enableSoftLimits() {
                rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
                rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                
            }

            @Override
            public Subsystem getSubsystemObject() {
                return new Subsystem() {
                    
                };
            }
        };
    }
}

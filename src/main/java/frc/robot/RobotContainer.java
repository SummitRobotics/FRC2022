// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.Lemonlight.LEDModes;
import frc.robot.devices.PDP;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.JoystickDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.Ports;
import frc.robot.utilities.lists.StatusPriorities;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandScheduler scheduler;

    private final ControllerDriver controller1;
    private final LaunchpadDriver launchpad;
    private final JoystickDriver joystick;

    private final Drivetrain drivetrain;

    private final Lemonlight targetingLimelight, ballDetectionLimelight;
    private final PDP pdp;
    private final AHRS gyro;

    private final Command teleInit;
    private final Command autoInit;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        scheduler = CommandScheduler.getInstance();

        controller1 = new ControllerDriver(Ports.XBOX_PORT);
        launchpad = new LaunchpadDriver(Ports.LAUNCHPAD_PORT);
        joystick = new JoystickDriver(Ports.JOYSTICK_PORT);
        pdp = new PDP();

        new LEDCall("disabled", LEDPriorities.ON, LEDRange.All).solid(Colors.DIM_GREEN).activate();
        ShuffleboardDriver.statusDisplay.addStatus(
            "default", "robot on", Colors.WHITE, StatusPriorities.ON);

        gyro = new AHRS();
        targetingLimelight = new Lemonlight("limelight");
        // TODO: need to ensure that this name is set on the limelight as well.
        ballDetectionLimelight = new Lemonlight("balldetect");

        drivetrain = new Drivetrain(gyro);

        autoInit = new SequentialCommandGroup(
                new InstantCommand(
                        () -> ShuffleboardDriver.statusDisplay.addStatus(
                            "auto",
                            "robot in auto",
                            Colors.TEAM,
                            StatusPriorities.ENABLED)),
                new InstantCommand(drivetrain::highGear),
                new InstantCommand(() -> {
                    launchpad.bigLEDRed.set(false);
                    launchpad.bigLEDGreen.set(true);
                }));

        teleInit =
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                        // simulated robots don't have joysticks
                        if (RobotBase.isReal()) {
                            if (!controller1.isConnected()
                                || !launchpad.isConnected()
                                || !joystick.isConnected()) {
                                System.out.println(
                                    "not enough joysticks connected,"
                                        + "please make sure the xbox controller,launchpad,"
                                        + "and joystick are connected to the driver-station");
                                throw (new RuntimeException("not enough joysticks connected"));
                            }
                            
                            if (!controller1.isXboxController()) {
                                System.out.println("controller 0 is not the xbox controller");
                                throw (new RuntimeException("incorrect joystick in port 0"));
                            }
                        }
                    }),
                new InstantCommand(() -> ShuffleboardDriver.statusDisplay.removeStatus("auto")),
                new InstantCommand(
                        () -> ShuffleboardDriver.statusDisplay.addStatus(
                            "enabled",
                            "robot enabled",
                            Colors.TEAM,
                            StatusPriorities.ENABLED)),
                new InstantCommand(() -> joystick.reEnableJoystickCalibrationCheck()),
                new InstantCommand(drivetrain::highGear),
                new InstantCommand(() -> targetingLimelight.setLEDMode(LEDModes.FORCE_OFF)),
                new InstantCommand(() -> ballDetectionLimelight.setLEDMode(LEDModes.FORCE_OFF)),
                new InstantCommand(() -> {
                    launchpad.bigLEDRed.set(false);
                    launchpad.bigLEDGreen.set(true);
                }));

        // Configure the button bindings
        setDefaultCommands();
        configureButtonBindings();

        // Init Telemetry
        initTelemetry();
    }

    private void setDefaultCommands() {
        // drive by controller
        drivetrain.setDefaultCommand(new ArcadeDrive(
            drivetrain,
            controller1.rightTrigger,
            controller1.leftTrigger,
            controller1.leftX));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this method to init all the subsystems' telemetry stuff.
     */
    private void initTelemetry() {
        SmartDashboard.putData("PDP", pdp);
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("Lemonlight", targetingLimelight);
        SmartDashboard.putData("Lemonlight", ballDetectionLimelight);
    }

    /**
     * runs when the robot gets disabled.
     */
    public void disabledInit() {
        LEDs.getInstance().removeAllCalls();
        new LEDCall("disabled", LEDPriorities.ON, LEDRange.All).solid(Colors.DIM_GREEN).activate();
        ShuffleboardDriver.statusDisplay.removeStatus("enabled");
        ChangeRateLimiter.resetAllChangeRateLimiters();
    }

    /**
     * runs when the robot is powered on.
     */
    public void robotInit() {
        ShuffleboardDriver.init();
    }
    
    /**
     * runs once every ~20ms when in teleop.
     */
    public void teleopPeriodic() {
    }

    /**
     * runs when robot is inited to teleop.
     */
    public void teleopInit() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}

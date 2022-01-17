// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.devices.Lemonlight.LEDModes;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.JoystickDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.oi.drivers.ShufhellboardDriver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.Ports;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.StatusPriorities;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	private CommandScheduler scheduler;

	private ControllerDriver controller1;
	private LaunchpadDriver launchpad;
	private JoystickDriver joystick;

	private Drivetrain drivetrain;
	private Shifter shifter;

	private Lemonlight limelight;
	private AHRS gyro;

	private Command teleInit;
	private Command autoInit;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		scheduler = CommandScheduler.getInstance();

		controller1 = new ControllerDriver(Ports.XBOX_PORT);
		launchpad = new LaunchpadDriver(Ports.LAUNCHPAD_PORT);
		joystick = new JoystickDriver(Ports.JOYSTICK_PORT);

		new LEDCall("disabled", LEDPriorities.on, LEDRange.All).solid(Colors.DimGreen).activate();
		ShufhellboardDriver.statusDisplay.addStatus("default", "robot on", Colors.White, StatusPriorities.on);

		gyro = new AHRS();
		limelight = new Lemonlight();

		shifter = new Shifter();
		drivetrain = new Drivetrain(gyro, () -> shifter.getShiftState());

		autoInit = new SequentialCommandGroup(
				new InstantCommand(
						() -> ShufhellboardDriver.statusDisplay.addStatus("auto", "robot in auto", Colors.Team,
								StatusPriorities.enabled)),
				new InstantCommand(shifter::highGear),
				new InstantCommand(() -> {
					launchpad.bigLEDRed.set(false);
					launchpad.bigLEDGreen.set(true);
				}));

		teleInit = new SequentialCommandGroup(
				new InstantCommand(() -> {
					// simulated robots dont have joysticks
					if (RobotBase.isReal()) {
						if (!controller1.isConnected() || !launchpad.isConnected() || !joystick.isConnected()) {
							System.out.println("not enough joysticks connected, please make sure the xbox controller, launchpad, and joystick are connected to the driverstation");
							throw (new RuntimeException("not enough joysticks connected"));
						}

						if (!controller1.isXboxControler()) {
							System.out.println("controller 0 is not the xbox controller");
							throw (new RuntimeException("incorrect joystick in port 0"));
						}
					}
				}),
				new InstantCommand(() -> ShufhellboardDriver.statusDisplay.removeStatus("auto")),
				new InstantCommand(
						() -> ShufhellboardDriver.statusDisplay.addStatus("enabled", "robot enabled", Colors.Team,
								StatusPriorities.enabled)),
				new InstantCommand(() -> joystick.ReEnableJoysticCalibrationCheck()),
				new InstantCommand(shifter::highGear),
				new InstantCommand(() -> limelight.setLEDMode(LEDModes.FORCE_OFF)),
				new InstantCommand(() -> {
					launchpad.bigLEDRed.set(false);
					launchpad.bigLEDGreen.set(true);
				}));

		// Configure the button bindings
		setDefaultCommands();
		configureButtonBindings();
	}

	private void setDefaultCommands() {
		// drive by controler
		drivetrain.setDefaultCommand(new ArcadeDrive(
				drivetrain,
				controller1.rightTrigger,
				controller1.leftTrigger,
				controller1.leftX,
				shifter::lowGear));

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}

	/**
	 * runs when the robot gets disabled
	 */
	public void disabledInit() {
		LEDs.getInstance().removeAllCalls();
		new LEDCall("disabled", LEDPriorities.on, LEDRange.All).solid(Colors.DimGreen).activate();
		ShufhellboardDriver.statusDisplay.removeStatus("enabled");
		ChangeRateLimiter.resetAllChangeRateLimiters();
	}

	/**
	 * runs once every ~20ms when in telyop
	 */
	public void teleopPeriodic() {
	}

	/**
	 * runs when robot is inited to telyop
	 */
	public void teleopInit() {
	}

	/**
	 * runs when the robot is powered on
	 */
	public void robotInit() {
		ShufhellboardDriver.init();
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

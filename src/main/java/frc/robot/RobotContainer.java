package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climb.ClimbAutomation;
import frc.robot.commands.climb.ClimbMO;
import frc.robot.commands.climb.ClimbManual;
import frc.robot.commands.climb.ClimbSemiAuto;
import frc.robot.commands.climb.ClimbAutomationBetter;
import frc.robot.commands.climb.climbAutomationSteps.AutoAlign;
import frc.robot.commands.climb.climbAutomationSteps.CycleArms;
import frc.robot.commands.climb.climbAutomationSteps.MoveArms;
import frc.robot.commands.conveyor.ConveyorAutomation;
import frc.robot.commands.conveyor.ConveyorMO;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveByTime;
import frc.robot.commands.drivetrain.FullAutoIntakeDrive;
import frc.robot.commands.drivetrain.DriveByTime;
import frc.robot.commands.homing.HomeByCurrent;
import frc.robot.commands.intake.DefaultIntake;
import frc.robot.commands.intake.IntakeMO;
import frc.robot.commands.intake.IntakeToggle;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.shooter.FullAutoShooterAssembly;
import frc.robot.commands.shooter.SemiAutoShooterAssembly;
import frc.robot.commands.shooter.ShooterAtStart;
import frc.robot.commands.shooter.ShooterMO;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.LidarV4;
import frc.robot.devices.PCM;
import frc.robot.devices.PDP;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.JoystickDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.Ports;
import frc.robot.utilities.lists.StatusPriorities;
import java.util.function.Supplier;

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

    private final CommandScheduler scheduler;
    private final ControllerDriver controller1;
    private final LaunchpadDriver launchpad;
    private final JoystickDriver joystick;

    // Subsystems
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Intake intake;
    private final Climb climb;

    private final Lemonlight
        targetingLimelight,
        ballDetectionLimelight;
    private final PCM pcm;
    private final AHRS gyro;
    private final PowerDistribution pdp;

    private final HomeByCurrent homeLeftArm;
    private final HomeByCurrent homeRightArm;

    private final ColorSensor colorSensor;
    private final LidarV4 lidar;
    private Command fullAutoShooterAssembly;
    private Supplier<Command> fullAutoIntake;
    private final Command teleInit;
    private final Command autoInit;
    private Command auto;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        scheduler = CommandScheduler.getInstance();
        controller1 = new ControllerDriver(Ports.XBOX_PORT);
        launchpad = new LaunchpadDriver(Ports.LAUNCHPAD_PORT);
        joystick = new JoystickDriver(Ports.JOYSTICK_PORT);
        pcm = new PCM(Ports.PCM_1);
        lidar = new LidarV4(0x62);
        colorSensor = new ColorSensor();
        pdp = new PowerDistribution(1, ModuleType.kRev);

        LEDs.getInstance().addCall("disabled", new LEDCall(LEDPriorities.ON, LEDRange.All).solid(Colors.DIM_GREEN));
        ShuffleboardDriver.statusDisplay.addStatus(
                "default", "robot on", Colors.WHITE, StatusPriorities.ON);

        gyro = new AHRS();
        
        targetingLimelight = new Lemonlight("gloworm", false, true);
        // TODO: need to ensure that this name is set on the limelight as well.
        ballDetectionLimelight = new Lemonlight("balldetect", true, true);

        // Init Subsystems
        drivetrain = new Drivetrain(gyro);
        shooter = new Shooter();
        conveyor = new Conveyor(colorSensor, lidar);
        intake = new Intake();
        climb = new Climb(gyro);

        // TODO - set these values
        homeLeftArm = new HomeByCurrent(climb.getLeftArmHomeable(), .2, 30, Climb.BACK_LIMIT, Climb.FORWARD_LIMIT);
        homeRightArm = new HomeByCurrent(climb.getRightArmHomeable(), .2, 30, Climb.BACK_LIMIT, Climb.FORWARD_LIMIT);
        
        autoInit = new SequentialCommandGroup(
                new InstantCommand(
                        () -> ShuffleboardDriver.statusDisplay.addStatus(
                                "auto",
                                "robot in auto",
                                Colors.TEAM,
                                StatusPriorities.ENABLED)),
                new InstantCommand(drivetrain::highGear),
                new InstantCommand(intake::lowerIntake),
                new InstantCommand(() -> {
                    launchpad.bigLEDRed.set(false);
                    launchpad.bigLEDGreen.set(true);
                })
                );

        teleInit = new SequentialCommandGroup(
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
                new InstantCommand(() -> pcm.enableCompressorDigital()),
                new InstantCommand(() -> ShuffleboardDriver.statusDisplay.removeStatus("auto")),
                new InstantCommand(
                        () -> ShuffleboardDriver.statusDisplay.addStatus(
                                "enabled",
                                "robot enabled",
                                Colors.TEAM,
                                StatusPriorities.ENABLED)),
                new InstantCommand(() -> joystick.reEnableJoystickCalibrationCheck()),
                new InstantCommand(drivetrain::highGear),
                // new InstantCommand(() -> targetingLimelight.setLEDMode(LEDModes.FORCE_OFF)),
                // new InstantCommand(() ->
                // ballDetectionLimelight.setLEDMode(LEDModes.FORCE_OFF)),

                new ParallelCommandGroup(homeLeftArm, homeRightArm),

                new InstantCommand(() -> {
                    launchpad.bigLEDRed.set(false);
                    launchpad.bigLEDGreen.set(false);
                }), 
                new RaiseIntake(intake)
                );

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
                controller1.leftX, 
                controller1.dPadAny));
        intake.setDefaultCommand(new DefaultIntake(intake, conveyor));
        conveyor.setDefaultCommand(new ConveyorAutomation(conveyor, intake, shooter));
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
        controller1.rightBumper.whenReleased(new InstantCommand(drivetrain::toggleShift));
        controller1.leftBumper.whenReleased(new InstantCommand(drivetrain::toggleShift));

        controller1.buttonA.whenPressed(new LowerIntake(intake));
        controller1.buttonB.whenPressed(
            new RaiseIntake(intake)
        );

        // Conveyor
        launchpad.buttonB.whileHeld(new ConveyorMO(conveyor, joystick.axisY, joystick.button2, joystick.button3));

        // Intake
        // controller1.buttonB.whenReleased(new IntakeToggle(intake));
        launchpad.buttonC.whileHeld(new IntakeMO(intake, joystick.axisY, joystick.button2));

        // Shooter
        ShooterMO shooterMO = new ShooterMO(shooter, joystick.axisZ, launchpad.buttonF, joystick.trigger);
        launchpad.buttonI.whileHeld(shooterMO);
        launchpad.buttonI.commandBind(shooterMO);
        // launchpad.funMiddle.whileHeld(new SemiAutoShooterAssembly(shooter, conveyor, drivetrain, targetingLimelight, joystick.trigger, joystick.axisY));

        //launchpad.buttonF.booleanSupplierBind(shooter::getHoodPos);


        //Climb
        ClimbMO climbMO = new ClimbMO(climb, joystick.axisY, joystick.button4,
                joystick.button5, joystick.button2, joystick.button7,
                joystick.button6, joystick.button8);

        launchpad.missileA.whileHeld(climbMO);
        
        // ClimbSemiAuto climbSemiAuto = new ClimbSemiAuto(climb, joystick.button2, joystick.button8, joystick.button4, joystick.button5, joystick.button3, joystick.button7);
        // launchpad.missileB.toggleWhenPressed(climbSemiAuto);
        ClimbAutomationBetter autoClimbCommand = new ClimbAutomationBetter(drivetrain, climb);
        launchpad.missileB.whileHeld(autoClimbCommand);


        ClimbManual climbManual = new ClimbManual(climb, joystick.axisY, joystick.button4,
                joystick.button5, joystick.button2, joystick.button7,
                joystick.button6, joystick.button8);

        launchpad.buttonA.whileHeld(climbManual);
        launchpad.buttonA.commandBind(climbManual);

        // ClimbAutomation climbAutomation = new ClimbAutomation(climb, drivetrain);
        // launchpad.buttonH.whileHeld(climbAutomation);
        // launchpad.buttonH.commandBind(climbAutomation);

        //launchpad.missileB.whileHeld(climbSemiAuto);
        //launchpad.buttonG.whileHeld(new ArcadeDrive(drivetrain, joystick.axisY, joystick.axisX, joystick.button2));
        // launchpad.buttonG.whenPressed(new InstantCommand(() -> climb.togglePivotPos()));
        // launchpad.buttonG.booleanSupplierBind(() -> climb.getPivotPos());

        //ClimbAutomation climbAutomation = new ClimbAutomation(climb, drivetrain, launchpad.buttonG);
        //launchpad.missileB.whileHeld(climbAutomation);
    }

    /**
     * Use this method to init all the subsystems' telemetry stuff.
     */
    private void initTelemetry() {
        //SmartDashboard.putData("PDP", pdp);
        SmartDashboard.putData("PCM", pcm);
        // SmartDashboard.putData("Drivetrain", drivetrain);
        // SmartDashboard.putData("Lemonlight", targetingLimelight);
        // SmartDashboard.putData("Lemonlight", ballDetectionLimelight);
        //SmartDashboard.putData("Shooter", shooter);
        //SmartDashboard.putData("Conveyor", conveyor);
        // SmartDashboard.putData("Intake", intake);
        // SmartDashboard.putData("Color Sensor", colorSensor);
        // SmartDashboard.putData("Lidar", lidar);
        //SmartDashboard.putData("Climb", climb);
    }

    /**
     * runs when the robot gets disabled.
     */
    public void disabledInit() {
        LEDs.getInstance().removeAllCalls();
        LEDs.getInstance().addCall("disabled", new LEDCall(LEDPriorities.ON, LEDRange.All).solid(Colors.DIM_GREEN));
        ShuffleboardDriver.statusDisplay.removeStatus("enabled");
        ChangeRateLimiter.resetAllChangeRateLimiters();
    }

    /**
     * runs when the robot is powered on.
     */
    public void robotInit() {
        gyro.calibrate();
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
        scheduler.schedule(teleInit);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // An ExampleCommand will run in autonomous
        //  // sets up all the splines so we dont need to spend lots of time
        // // turning the json files into trajectorys when we want to run them
        // String ball1 = "paths\1.path";
        // try {
        //     Command fball1 = Functions.splineCommandFromFile(drivetrain, ball1);
        //     // possible 4 ball auto
        //     auto = new SequentialCommandGroup(
        //             autoInit,
        //             new PrintCommand("paiosuibsfub"),
        //             new ShooterAtStart(shooter, conveyor).withTimeout(10),
        //             new PrintCommand("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa"),
        //             fball1
        //             // fullAutoShooterAssembly,
        //             // fullAutoIntake.get(),
        //             // fullAutoShooterAssembly,
        //             // fullAutoIntake.get(),
        //             // fullAutoShooterAssembly
        //             );

        //     return auto;
        // } catch (Exception e) {
        //     System.out.println("An error occured when making autoInit: " + e);
        // }

        return new SequentialCommandGroup(
            //autoInit,
            //new ShooterAtStart(shooter, conveyor).withTimeout(10),
            //new InstantCommand(() -> {drivetrain.setLeftMotorPower(-0.3); drivetrain.setRightMotorPower(-0.3);}),
            new DriveByTime(drivetrain, 3, -0.3)
            //new InstantCommand(() -> drivetrain.stop())
            // fullAutoShooterAssembly,
            // fullAutoIntake.get(),
            // fullAutoShooterAssembly,
            // fullAutoIntake.get(),
            // fullAutoShooterAssembly
            );
        
    }
}
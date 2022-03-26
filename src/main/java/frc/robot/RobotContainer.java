package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DriveToHub;
import frc.robot.commands.climb.*;
import frc.robot.commands.conveyor.ConveyorAutomation;
import frc.robot.commands.conveyor.ConveyorMO;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.homing.HomeByCurrent;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.devices.*;
import frc.robot.devices.LEDs.*;
import frc.robot.oi.drivers.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.lists.*;
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

    private final Lemonlight targetingLimelight,
            ballDetectionLimelight;
    private final PCM pcm;
    private final AHRS gyro;
    private final PowerDistribution pdp;

    private final Supplier<Command> homeArms;



    private final ColorSensor colorSensor;
    private final LidarV4 lidar;
    private final Command teleInit;
    private ClimbAutomationBetter autoClimbCommand;
    private Command arcadeDrive;

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

        targetingLimelight = new Lemonlight("limelight-target", false, false);
        // TODO: need to ensure that this name is set on the limelight as well.
        ballDetectionLimelight = new Lemonlight("limelight-balls", true, false);

        // Init Subsystems
        drivetrain = new Drivetrain(gyro);
        shooter = new Shooter();
        conveyor = new Conveyor(colorSensor, lidar);
        intake = new Intake();
        climb = new Climb(gyro);

        arcadeDrive = new ArcadeDrive(
            drivetrain,
            intake,
            controller1.rightTrigger,
            controller1.leftTrigger,
            controller1.leftX,
            controller1.dPadAny);

        homeArms = ()-> new ParallelCommandGroup(new HomeByCurrent(climb.getLeftArmHomeable(), .2, 30, Climb.BACK_LIMIT, Climb.FORWARD_LIMIT), new HomeByCurrent(climb.getRightArmHomeable(), .2, 30, Climb.BACK_LIMIT, Climb.FORWARD_LIMIT));


        autoClimbCommand = new ClimbAutomationBetter(drivetrain, climb);

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

                homeArms.get(),

                new InstantCommand(() -> {
                    launchpad.bigLEDRed.set(false);
                    launchpad.bigLEDGreen.set(false);
                }),
                new RaiseIntake(intake));

        // Configure the button bindings
        setDefaultCommands();
        configureButtonBindings();

        // Init Telemetry
        initTelemetry();
    }

    private void setDefaultCommands() {
        // drive by controller
        drivetrain.setDefaultCommand(arcadeDrive);
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
        ShuffleboardDriver.homeArms.commandBind(homeArms.get(), ShuffleboardDriver.homeArms::whenPressed);

        controller1.rightBumper.whenReleased(new InstantCommand(drivetrain::toggleShift));
        controller1.leftBumper.whenReleased(new InstantCommand(drivetrain::toggleShift));

        controller1.buttonA.whenPressed(new LowerIntake(intake));
        controller1.buttonB.whenPressed(
                new RaiseIntake(intake));

        controller1.buttonX.whileHeld(new FullAutoIntake(drivetrain, intake, ballDetectionLimelight, conveyor).withTimeout(5));
        // FullAutoShooterAssembly fullAutoShooterAssembly = new FullAutoShooterAssembly(shooter, conveyor, drivetrain, targetingLimelight);
        // launchpad.buttonG.whileHeld(fullAutoShooterAssembly);
        // launchpad.buttonG.commandBind(fullAutoShooterAssembly);
        // Conveyor
        launchpad.buttonB.whileHeld(new ConveyorMO(conveyor, joystick.axisY, joystick.button2, joystick.button3));

        // Intake
        // controller1.buttonB.whenReleased(new IntakeToggle(intake));
        launchpad.buttonC.whileHeld(new IntakeMO(intake, joystick.axisY, joystick.button2));

        // Shooter
        ShooterMO shooterMO = new ShooterMO(shooter, joystick.axisZ, launchpad.buttonF, joystick.trigger);
        launchpad.buttonI.whileHeld(shooterMO);
        launchpad.buttonI.commandBind(shooterMO);
        // launchpad.funMiddle.whileHeld(new SemiAutoShooterAssembly(shooter, conveyor,
        // drivetrain, targetingLimelight, joystick.trigger, joystick.axisY));

        // launchpad.buttonF.booleanSupplierBind(shooter::getHoodPos);

        // Climb
        ClimbMO climbMO = new ClimbMO(climb, joystick.axisY, joystick.button4,
                joystick.button5, joystick.button2, joystick.button7,
                joystick.button6, joystick.button8);

        launchpad.missileA.whileHeld(climbMO);

        // ClimbSemiAuto climbSemiAuto = new ClimbSemiAuto(climb, joystick.button2,
        // joystick.button8, joystick.button4, joystick.button5, joystick.button3,
        // joystick.button7);
        // launchpad.missileB.toggleWhenPressed(climbSemiAuto);
        launchpad.missileB.whileHeld(autoClimbCommand);
        launchpad.buttonG.whileHeld(new ShooterLow(shooter, joystick.trigger));

        ClimbManual climbManual = new ClimbManual(climb, joystick.axisY, joystick.button4,
                joystick.button5, joystick.button2, joystick.button7,
                joystick.button6, joystick.button8);

        launchpad.buttonA.whenHeld(climbManual);
        launchpad.buttonA.commandBind(climbManual);

        Command sas = new SemiAutoShooterAssembly(shooter, conveyor, drivetrain, targetingLimelight, joystick.trigger,
                joystick.axisY, arcadeDrive);
        launchpad.buttonH.whileHeld(sas);
        launchpad.buttonH.commandBind(sas);

        // launchpad.missileB.whileHeld(climbSemiAuto);
        // launchpad.buttonG.whileHeld(new ArcadeDrive(drivetrain, joystick.axisY,
        // joystick.axisX, joystick.button2));
        // launchpad.buttonG.whenPressed(new InstantCommand(() ->
        // climb.togglePivotPos()));
        // launchpad.buttonG.booleanSupplierBind(() -> climb.getPivotPos());

        // ClimbAutomation climbAutomation = new ClimbAutomation(climb, drivetrain,
        // launchpad.buttonG);
        // launchpad.missileB.whileHeld(climbAutomation);
    }

    /**
     * Use this method to init all the subsystems' telemetry stuff.
     */
    private void initTelemetry() {
        // SmartDashboard.putData("PDP", pdp);
        SmartDashboard.putData("PCM", pcm);
        // SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("Lemonlight", targetingLimelight);
        SmartDashboard.putData("Lemonlight", ballDetectionLimelight);
        // SmartDashboard.putData("Shooter", shooter);
        // SmartDashboard.putData("Conveyor", conveyor);
        // SmartDashboard.putData("Intake", intake);
        // SmartDashboard.putData("Color Sensor", colorSensor);
        // SmartDashboard.putData("Lidar", lidar);
        // SmartDashboard.putData("Climb", climb);
    }

    /**
     * runs when the robot gets disabled.
     */
    public void disabledInit() {
        autoClimbCommand.resetCommandState();
        LEDs.getInstance().removeAllCalls();
        LEDs.getInstance().addCall("disabled", new LEDCall(LEDPriorities.ON, LEDRange.All).solid(Colors.DIM_GREEN));
        ShuffleboardDriver.statusDisplay.removeStatus("enabled");
        ChangeRateLimiter.resetAllChangeRateLimiters();
    }

    /**
     * runs when the robot is powered on.
     */
    public void robotInit() {
        createAutoCommands();
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

    private void createAutoCommands(){

        Supplier<Command> autoInit = () -> new SequentialCommandGroup(
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
            }));

        Command deafultAuto = new SequentialCommandGroup(
            autoInit.get(),
            new ShooterAtStart(shooter, conveyor),
            new DriveByTime(drivetrain, 1.5, -0.5),
            new PrintCommand("auto done"));

        ShuffleboardDriver.autoChooser.setDefaultOption("shoot and drive", deafultAuto);

        Command shootWaitDrive = new SequentialCommandGroup(
            autoInit.get(),
            new ShooterAtStart(shooter, conveyor),
            new WaitCommand(8),
            new DriveByTime(drivetrain, 1.5, -0.5),
            new PrintCommand("auto done")
        );

        ShuffleboardDriver.autoChooser.addOption("shoot, wait, drive", shootWaitDrive);

        Command shootOnly = new SequentialCommandGroup(
            autoInit.get(),
            new ShooterAtStart(shooter, conveyor),
            new PrintCommand("auto done")
        );

        ShuffleboardDriver.autoChooser.addOption("shoot only", shootOnly);


        Command driveOlnly = new SequentialCommandGroup(
                autoInit.get(),
                new WaitCommand(10),
                new DriveByTime(drivetrain, 1.5, -0.5),
                new PrintCommand("auto done")
        );

        ShuffleboardDriver.autoChooser.addOption("drive only", driveOlnly);


        
        Command twoBallAuto = new SequentialCommandGroup(
            autoInit.get(),
            new LowerIntake(intake),
            new FullAutoIntake(drivetrain, intake, ballDetectionLimelight, conveyor),
            new WaitCommand(1),
            new TurnByEncoder(190, drivetrain),
            new RaiseIntake(intake),
            new WaitCommand(.5),
            new DriveToHub(drivetrain, targetingLimelight).withTimeout(3),
            new ShooterAtStart(shooter, conveyor, 1000),
            new PrintCommand("auto done"));

        ShuffleboardDriver.autoChooser.addOption("2 ball", twoBallAuto);


        Command twoBallAutoDLC = new SequentialCommandGroup(
            //2 ball
            autoInit.get(),
            new LowerIntake(intake),
            new EncoderDrive(1, 1, drivetrain),
            new WaitCommand(1),
            new TurnByEncoder(180, drivetrain),
            new WaitCommand(1),
            new EncoderDrive(2, 2, drivetrain),
            new WaitCommand(1),
            new TurnByEncoder(110, drivetrain),
            new WaitCommand(1),
            new ShooterAtStart(shooter, conveyor, 1200),
            //new stuff
            new TurnByEncoder(60, drivetrain),
            new EncoderDrive(10, 10, drivetrain),
            new TurnByEncoder(35, drivetrain),
            new EncoderDrive(.2, .2, drivetrain),
            new WaitCommand(3),
            new RaiseIntake(intake),
            new TurnByEncoder(180, drivetrain),
            new EncoderDrive(10, 10, drivetrain),
            new FullAutoShooterAssembly(shooter, conveyor, drivetrain, targetingLimelight),
            new PrintCommand("auto done"));

        ShuffleboardDriver.autoChooser.addOption("4 ball", twoBallAutoDLC);

    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        System.out.println("Called");
        // // An ExampleCommand will run in autonomous
        //  // sets up all the splines so we dont need to spend lots of time
        // // turning the json files into trajectorys when we want to run them
        // String ball1 = "paths/output/circle.wpilib.json";
        // try {
        //     Command fball1 = Functions.splineCommandFromFile(drivetrain, ball1);
        //     auto = new SequentialCommandGroup(
        //         autoInit.get(),
        //         fball1);
        // } catch (Exception e) {
        //     System.out.println("Spline error occurred: " + e);
        // }






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

        

        return ShuffleboardDriver.autoChooser.getSelected();
        
    }
}
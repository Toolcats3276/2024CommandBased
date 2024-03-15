package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.ShootCommand.AutoStartShotCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.AutoStartShotCoCommand2;
import frc.robot.commands.AutoCommands.ShootCommand.PassOffCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.CloseMidNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.LeftNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.MidNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.RightNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoFarShotCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoFarShotCoCommand2;
import frc.robot.commands.AutoCommands.AutoSuckBackCommand;
import frc.robot.commands.AutoCommands.AmpCommands.AutoAmpCoCommand;
import frc.robot.commands.AutoCommands.AmpCommands.AutoInverseAmpCoCommand;
import frc.robot.commands.CompoundCommand.CancelCoCommand;
import frc.robot.commands.CompoundCommand.CompCoCommands.CompCoCommand;
import frc.robot.commands.CompoundCommand.CompCoCommands.ToggleCompCoCommand;
import frc.robot.commands.CompoundCommand.InfeedCoCommands.InfeedSensorCoCommand;
import frc.robot.commands.CompoundCommand.InfeedCoCommands.InfeedSensorCoCommand2;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.AutoTrapCoCommand;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.InverseScoreCommand;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.ScoringCoCommand;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.ShuttleCoCommand;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.SpeakerShotCoCommand;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.TrapCoCommand;
import frc.robot.commands.CompoundCommand.ScoringCoCommands.AmpCommands.ToggleAmpCoCommand;
import frc.robot.commands.z_ClimberCommands.BothManualCommands.ClimberDownCommand;
import frc.robot.commands.z_ClimberCommands.BothManualCommands.ClimberStopCommand;
import frc.robot.commands.z_ClimberCommands.BothManualCommands.ClimberUpCommand;
import frc.robot.commands.z_ClimberCommands.LeftManualCommands.LeftClimberDownCommand;
import frc.robot.commands.z_ClimberCommands.LeftManualCommands.LeftClimberStopCommand;
import frc.robot.commands.z_ClimberCommands.LeftManualCommands.LeftClimberUpCommand;
import frc.robot.commands.z_ClimberCommands.RightManualCommands.RightClimberDownCommand;
import frc.robot.commands.z_ClimberCommands.RightManualCommands.RightClimberStopCommand;
import frc.robot.commands.z_ClimberCommands.RightManualCommands.RightClimberUpCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    // CREATING NEW CONTROLLER OBJECTS
    private final Joystick m_DriveController = new Joystick(0);
    private final XboxController m_CoXboxController = new XboxController(1); 
    private final XboxController m_CoFlightStick = new XboxController(2); 
    
    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;
    private final int rotationAxis = Joystick.AxisType.kZ.value;

    private final int ArmAxis = XboxController.Axis.kLeftY.value;
    private final int WristAxis = XboxController.Axis.kRightY.value;

    // CREATING m_DriveController BUTTONS
    private final JoystickButton zeroGyro = new JoystickButton(m_DriveController, 14);
    private final JoystickButton robotCentric = new JoystickButton(m_DriveController, 11);

    private final JoystickButton Drive = new JoystickButton(m_DriveController, 2);
    private final JoystickButton Amp = new JoystickButton(m_DriveController, 4);
    private final JoystickButton Infeed = new JoystickButton(m_DriveController, 7);
    private final JoystickButton Infeed2 = new JoystickButton(m_DriveController, 6);

    private final JoystickButton Shoot = new JoystickButton(m_DriveController, 1);
    // private final JoystickButton HighShot = new JoystickButton(m_DriveController, 10);
    private final JoystickButton InverseShot = new JoystickButton(m_DriveController, 10);

    private final JoystickButton Shuttle = new JoystickButton(m_DriveController, 3);

    private final JoystickButton Trap = new JoystickButton(m_DriveController, 9);

    private final JoystickButton ManualOutfeed = new JoystickButton(m_DriveController, 8);

    private final JoystickButton DefenceShot = new JoystickButton(m_DriveController, 15);

    private final JoystickButton Cancel = new JoystickButton(m_DriveController, 5);

    
    // CREATING m_CoXboxController BUTTONS
    private final JoystickButton CoManualInfeed = new JoystickButton(m_CoXboxController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CoManualOutfeed = new JoystickButton(m_CoXboxController, XboxController.Button.kRightBumper.value);

    // private final JoystickButton CoTest = new JoystickButton(m_CoXboxController, XboxController.Button.kA.value);
    private final JoystickButton CoZeroGyro = new JoystickButton(m_CoXboxController, XboxController.Button.kY.value);
    private final JoystickButton CoDefenceShot = new JoystickButton(m_CoXboxController, XboxController.Button.kA.value);

    private final JoystickButton CoCancel = new JoystickButton(m_CoXboxController, XboxController.Button.kB.value);

    // CREATING m_CoFlightStick
    private final JoystickButton BothClimberUp = new JoystickButton(m_CoFlightStick, 1);
    private final JoystickButton BothClimberDown = new JoystickButton(m_CoFlightStick, 2);

    private final JoystickButton LeftClimberUp = new JoystickButton(m_CoFlightStick, 5);
    private final JoystickButton LeftClimberDown = new JoystickButton(m_CoFlightStick, 6);

    private final JoystickButton RightClimberUp = new JoystickButton(m_CoFlightStick, 3);
    private final JoystickButton RightClimberDown = new JoystickButton(m_CoFlightStick, 4);

    // CREATING NEW SUBSYSTEM OBJECTS
    private final Swerve s_Swerve = new Swerve();
    private final ArmSS s_Arm = new ArmSS();
    private final ClimberSS s_Climber = new ClimberSS();
    private final WristSS s_Wrist = new WristSS();
    private final InfeedSS s_Infeed = new InfeedSS();
    private final ShooterSS s_Shooter = new ShooterSS();
    private final SensorSS s_Sensor = new SensorSS();
    private final LEDSS s_LED = new LEDSS();


    //THE CONTAINER FOR THE ROBOT.  CONTAINS SUBSYSTEMS, OPERATOR INTERFACE DEVICES. AND DEFAULT COMMANDS

    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -(m_DriveController.getRawAxis(translationAxis)), 
                () -> -(m_DriveController.getRawAxis(strafeAxis)),
                () -> -(m_DriveController.getRawAxis(rotationAxis)),
                () -> robotCentric.getAsBoolean()
            )
            
        );

        // s_Arm.setDefaultCommand(
        //     new ArmManualCommand(
        //         s_Arm, 
        //         () -> -m_CoXboxController.getRawAxis(ArmAxis))
        // );        

        // s_Wrist.setDefaultCommand(
        //     new WristManualCmd(
        //         s_Wrist,
        //         () -> -m_CoXboxController.getRawAxis(WristAxis))
        // );

        // s_Climber.setDefaultCommand(
        //     new ClimberStopCommand(s_Climber)
        // );


        // registering commands into path planner
        NamedCommands.registerCommand("ZeroGyro", new InstantCommand(() -> s_Swerve.zeroHeading()));

        NamedCommands.registerCommand("InfeedCommand", new AutoSuckBackCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter)
            .until(() -> s_Sensor.isDebounced()));
        NamedCommands.registerCommand("AutoInfeedCommand", new InfeedSensorCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
            .until(() -> s_Sensor.autoInfeedDelay()));

        NamedCommands.registerCommand("CompCommand", new CompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));

        NamedCommands.registerCommand("StartShot", new AutoStartShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist)
            .until(() -> !s_Sensor.shootDelay()));
        NamedCommands.registerCommand("StartShot2", new AutoStartShotCoCommand2(s_Infeed, s_Shooter, s_Arm, s_Wrist)
            .until(() -> !s_Sensor.shootDelay()));

        NamedCommands.registerCommand("SpeakerShot", new SpeakerShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("Amp", new AutoAmpCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));
        NamedCommands.registerCommand("InverseAmp", new AutoInverseAmpCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));

        NamedCommands.registerCommand("MidNoteShot", new MidNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("CloseMidNoteShot", new CloseMidNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("LeftNoteShot", new LeftNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("RightNoteShot", new RightNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("FarShot", new AutoFarShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("FarShot2", new AutoFarShotCoCommand2(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("PassOff", new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("TrapCommand", new TrapCoCommand(s_Wrist, s_Arm));

        

        // initializing autochooser and putting it on smartdachboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);



        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    // bind buttons to commands
    private void configureButtonBindings() {

        // m_DriveController Buttons
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        Cancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Climber, s_Sensor,
            () -> -m_CoXboxController.getRawAxis(WristAxis), 
            () -> -m_CoXboxController.getRawAxis(ArmAxis)));


        Drive.onTrue(new ToggleCompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));

        Infeed.onTrue(new InfeedSensorCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
            .until(() -> s_Sensor.isDebounced()));
        Infeed2.onTrue(new InfeedSensorCoCommand2(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
            .until(() -> s_Sensor.isDebounced()));

        // Infeed.onTrue(new InfeedCoCommand(s_Wrist, s_Arm, s_Infeed));
        Amp.onTrue(new ToggleAmpCoCommand(s_Wrist, s_Arm, s_Infeed));
        // AmpTwo.onTrue(new AmpTwoCoCommand(s_Wrist, s_Arm, s_Infeed));
        
        Shoot.onTrue(new ScoringCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist)
            .until(() -> !s_Sensor.shootDelay()));
        // FarShot.onTrue(new FarShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        InverseShot.onTrue(new InverseScoreCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist)
            .until(() -> !s_Sensor.shootDelay()));


        Shuttle.onTrue(new ShuttleCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED));
            // .until(() -> s_Sensor.isDebounced()));
        // HighShot.onTrue(new HighScoreCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        
        // Trap.onTrue(new TrapCoCommand(s_Wrist, s_Arm));
        Trap.onTrue(new AutoTrapCoCommand(s_Wrist, s_Arm, s_Climber));
        ManualOutfeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.OUTFEED));

        DefenceShot.onTrue(new PathPlannerAuto("DefenceShot"));

        //m_CoXboxBoxController buttons

        CoManualInfeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.INFEED_SPEED));
        CoManualOutfeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.OUTFEED));

        CoDefenceShot.onTrue(new PathPlannerAuto("DefenceShot"));

        // CoHighShot.onTrue(new HighScoreCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        CoZeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        CoCancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Climber, s_Sensor,
            () -> -m_CoXboxController.getRawAxis(WristAxis), 
            () -> -m_CoXboxController.getRawAxis(ArmAxis)));

        BothClimberUp.whileTrue(new ClimberUpCommand(s_Climber, s_Sensor))
            .onFalse(new ClimberStopCommand(s_Climber));
        BothClimberDown.whileTrue(new ClimberDownCommand(s_Climber))
            .onFalse(new ClimberStopCommand(s_Climber));

        LeftClimberUp.whileTrue(new LeftClimberUpCommand(s_Climber, s_Sensor))
            .onFalse(new LeftClimberStopCommand(s_Climber));
        LeftClimberDown.whileTrue(new LeftClimberDownCommand(s_Climber))
            .onFalse(new LeftClimberStopCommand(s_Climber));

        RightClimberUp.whileTrue(new RightClimberUpCommand(s_Climber, s_Sensor))
            .onFalse(new RightClimberStopCommand(s_Climber));
        RightClimberDown.whileTrue(new RightClimberDownCommand(s_Climber))
            .onFalse(new RightClimberStopCommand(s_Climber));


        // TESTING BUTTONS
        //########################################################################
            // Infeed.onTrue(new ArmPIDCommand(s_Arm, ArmConstants.INFEED_POS, 0.1))
            //     .onTrue(new WristPIDCommand(s_Wrist, WristConstants.INFEED_POS, 0.1));
            // Amp.onTrue(new ArmPIDCommand(s_Arm, ArmConstants.AMP_POSE, 0.1))
            //     .onTrue(new WristPIDCommand(s_Wrist, WristConstants.AMP_POS, 0.1));

            // CoTest.onTrue(new HighScoreCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        //########################################################################
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     // have it return the autochooser selection
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new PathPlannerAuto("Source-Far");
        // return new PathPlannerAuto("MidFourNote");
        // return new PathPlannerAuto("Amp-Far");
        return autoChooser.getSelected();
    }
}

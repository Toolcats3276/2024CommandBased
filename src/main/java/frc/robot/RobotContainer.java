package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.autos.AroundStageSmartAuto;
import frc.robot.autos.UnderStageSmartAuto;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.TeleopSwerve;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.BothManualCommands.ClimberDownCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.BothManualCommands.ClimberStopCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.BothManualCommands.ClimberUpCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.LeftManualCommands.LeftClimberDownCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.LeftManualCommands.LeftClimberStopCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.LeftManualCommands.LeftClimberUpCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.RightManualCommands.RightClimberDownCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.RightManualCommands.RightClimberStopCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.RightManualCommands.RightClimberUpCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.*;
import frc.robot.commands.TeleopCommands.CompoundCommand.CompCoCommands.CompCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.CompCoCommands.ToggleCompCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCompCoCommand;
// import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.InverseScoreCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.PassOffCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ScoringCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.SpeakerShotCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.TestShotCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.AmpCommands.ToggleAmpCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.InfeedScoringCommands.InfeedShootCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.InfeedScoringCommands.InverseScoreCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands.ToggleShuttleCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands.ToggleShuttleStateCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.TrapCommands.AutoTrapCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.TrapCommands.TrapCoCommand;
import frc.robot.commands.AutoCommands.AutoInfeedCoCommand;
import frc.robot.commands.AutoCommands.AutoSourceInfeedCoCommand;
import frc.robot.commands.AutoCommands.AmpCommands.AutoAmpCoCommand;
import frc.robot.commands.AutoCommands.AmpCommands.AutoInverseAmpCoCommand;
import frc.robot.commands.AutoCommands.OptimizedCommands.OpAutoInfeedCoCommand;
import frc.robot.commands.AutoCommands.OptimizedCommands.ShootCommands.OpAutoCloseMidShootCoCommands;
import frc.robot.commands.AutoCommands.OptimizedCommands.ShootCommands.OpAutoLeftShootCoCommands;
import frc.robot.commands.AutoCommands.OptimizedCommands.ShootCommands.OpAutoMidShootCoCommands;
import frc.robot.commands.AutoCommands.OptimizedCommands.ShootCommands.OpAutoRightShootCoCommands;
import frc.robot.commands.AutoCommands.ShootCommand.AutoStartShotCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.AutoStartShotCoCommand2;
import frc.robot.commands.AutoCommands.ShootCommand.AutoStartShotCoCommand3;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.CloseMidNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.LeftNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.MidNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands.RightNoteShootCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoFarShotCoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoFarShotCoCommand2;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoFarShotCoCommand3;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoSourceFarShot2CoCommand;
import frc.robot.commands.AutoCommands.ShootCommand.FarShot.AutoSourceFarShotCoCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> AutoChooser;

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
    private final JoystickButton robotCentric = new JoystickButton(m_DriveController, 0);
    
    private final JoystickButton Comp = new JoystickButton(m_DriveController, 2);
    private final JoystickButton SlowComp = new JoystickButton(m_DriveController, 16);
    private final JoystickButton Amp = new JoystickButton(m_DriveController, 4);

    private final JoystickButton Infeed = new JoystickButton(m_DriveController, 7);
    private final JoystickButton Infeed2 = new JoystickButton(m_DriveController, 8);

    private final JoystickButton ManualOutfeed = new JoystickButton(m_DriveController, 6);

    private final JoystickButton Shoot = new JoystickButton(m_DriveController, 1);
    // private final JoystickButton HighShot = new JoystickButton(m_DriveController, 10);
    private final JoystickButton InverseShot = new JoystickButton(m_DriveController, 9);

    private final JoystickButton Shuttle = new JoystickButton(m_DriveController, 3);

    private final JoystickButton Trap = new JoystickButton(m_DriveController, 10);
    // private final JoystickButton CoTrap = new JoystickButton(m_CoXboxController, XboxController.Button.kBack.value);
    private final JoystickButton AutoTrap = new JoystickButton(m_DriveController, 15);

    // private final JoystickButton DefenceShot = new JoystickButton(m_DriveController, 15);

    private final JoystickButton Cancel = new JoystickButton(m_DriveController, 5);

    
    // CREATING m_CoXboxController BUTTONS
    private final JoystickButton CoManualInfeed = new JoystickButton(m_CoXboxController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CoManualOutfeed = new JoystickButton(m_CoXboxController, XboxController.Button.kRightBumper.value);

    // private final JoystickButton CoTest = new JoystickButton(m_CoXboxController, XboxController.Button.kA.value);
    // private final JoystickButton CoZeroGyro = new JoystickButton(m_CoXboxController, XboxController.Button.kA.value);
    private final JoystickButton CoDefenceShot = new JoystickButton(m_CoXboxController, XboxController.Button.kX.value);
    private final JoystickButton CoPodiumShot = new JoystickButton(m_CoXboxController, XboxController.Button.kY.value);
    private final JoystickButton CoAmpShot = new JoystickButton(m_CoXboxController, XboxController.Button.kA.value);

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

        // registering commands into path planner
        NamedCommands.registerCommand("ZeroGyro", new InstantCommand(() -> s_Swerve.zeroHeading()));

        NamedCommands.registerCommand("InfeedCommand", new AutoInfeedCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter)
            .until(() -> s_Sensor.infeedDelay()));
        NamedCommands.registerCommand("OpInfeedCommand", new OpAutoInfeedCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter)
            .until(() -> s_Sensor.isTriggered()));
        NamedCommands.registerCommand("SourceInfeedCommand", new AutoSourceInfeedCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter)
            .until(() -> s_Sensor.infeedDelay()));

        NamedCommands.registerCommand("CompCommand", new CompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, ArmConstants.MAX_PID_OUTPUT, WristConstants.MAX_PID_OUTPUT));

        NamedCommands.registerCommand("StartShot", new AutoStartShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("StartShot2", new AutoStartShotCoCommand2(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("StartShot3", new AutoStartShotCoCommand3(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        // NamedCommands.registerCommand("SpeakerShot", new SpeakerShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("Amp", new AutoAmpCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));
        NamedCommands.registerCommand("InverseAmp", new AutoInverseAmpCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));

        NamedCommands.registerCommand("MidNoteShot", new MidNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("CloseMidNoteShot", new CloseMidNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("LeftNoteShot", new LeftNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("RightNoteShot", new RightNoteShootCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("OpMidNoteShot", new OpAutoMidShootCoCommands(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("OpCloseMidNoteShot", new OpAutoCloseMidShootCoCommands(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("OpLeftNoteShot", new OpAutoLeftShootCoCommands(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("OpRightNoteShot", new OpAutoRightShootCoCommands(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("FarShot", new AutoFarShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("FarShot2", new AutoFarShotCoCommand2(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("FarShot3", new AutoFarShotCoCommand3(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("SourceFarShot", new AutoSourceFarShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        NamedCommands.registerCommand("SourceFarShot2", new AutoSourceFarShot2CoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        
        NamedCommands.registerCommand("PassOff", new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        NamedCommands.registerCommand("TrapCommand", new TrapCoCommand(s_Wrist, s_Arm));

        

        // initializing autochooser and putting it on smartdashboard
        autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        AutoChooser = new SendableChooser<Command>();
        AutoChooser.setDefaultOption("None", new PrintCommand(" No Auto Selected"));
        AutoChooser.addOption("Five Note", new PathPlannerAuto("FiveNote"));
        AutoChooser.addOption("Four Note", new PathPlannerAuto("FourNote"));
        AutoChooser.addOption("Under Stage Source", new UnderStageSmartAuto(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Sensor));
        AutoChooser.addOption("Around Stage Source", new AroundStageSmartAuto(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Sensor));
        SmartDashboard.putData("Auto Chooser", AutoChooser);

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

        Cancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Climber, s_Sensor, s_LED,
            () -> -m_CoXboxController.getRawAxis(WristAxis), 
            () -> -m_CoXboxController.getRawAxis(ArmAxis)));


        Comp.onTrue(new ToggleCompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, ArmConstants.MAX_PID_OUTPUT, WristConstants.MAX_PID_OUTPUT));
        SlowComp.onTrue(new CompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, ArmConstants.SLOW_PID_OUTPUT, WristConstants.SLOW_PID_OUTPUT));

        Infeed.onTrue(new InfeedCompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
            .until(() -> s_Sensor.infeedDelay()));
        Infeed2.onTrue(new InfeedShootCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
            .until(() -> s_Sensor.infeedShotDelay()));

        Amp.onTrue(new ToggleAmpCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter));
        
        Shoot.onTrue(new ScoringCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist, s_Sensor));
        // Shoot.onTrue(new TestShotCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        InverseShot.onTrue(new InverseScoreCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
            .until(() -> s_Sensor.inverseDelay()));

        Shuttle.onTrue(new ToggleShuttleCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Sensor, s_LED));
        Shuttle.onTrue(new ToggleShuttleStateCoCommand(s_Sensor));

        // Shuttle.onTrue(new ShuttleCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED));
        // HighShot.onTrue(new HighScoreCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));
        
        Trap.onTrue(new TrapCoCommand(s_Wrist, s_Arm));
        AutoTrap.onTrue(new AutoTrapCoCommand(s_Wrist, s_Arm, s_Climber, s_Swerve, s_Sensor, s_Infeed));
        ManualOutfeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.OUTFEED));

        // DefenceShot.onTrue(new PathPlannerAuto("DefenceShot"));

        //m_CoXboxBoxController buttons

        CoManualInfeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.INFEED_SPEED));
        CoManualOutfeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.OUTFEED));

        CoDefenceShot.onTrue(new PathPlannerAuto("DefenceShot"));
        CoPodiumShot.onTrue(new PathPlannerAuto("DefenceShot2"));
        CoAmpShot.onTrue(new PathPlannerAuto("DefenceShot3"));

        // CoHighShot.onTrue(new HighScoreCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist));

        CoCancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, s_Climber, s_Sensor, s_LED,
            () -> -m_CoXboxController.getRawAxis(WristAxis), 
            () -> -m_CoXboxController.getRawAxis(ArmAxis)));

        // CoTrap.onTrue(new TrapCoCommand(s_Wrist, s_Arm));

        BothClimberUp.onTrue(new ClimberUpCommand(s_Climber, s_Sensor).handleInterrupt(() -> new ClimberStopCommand(s_Climber)))
            .onFalse(new ClimberStopCommand(s_Climber));
        BothClimberDown.onTrue(new ClimberDownCommand(s_Climber).handleInterrupt(() -> new ClimberStopCommand(s_Climber)))
            .onFalse(new ClimberStopCommand(s_Climber));

        LeftClimberUp.onTrue(new LeftClimberUpCommand(s_Climber, s_Sensor).handleInterrupt(() -> new ClimberStopCommand(s_Climber)))
            .onFalse(new LeftClimberStopCommand(s_Climber));
        LeftClimberDown.onTrue(new LeftClimberDownCommand(s_Climber).handleInterrupt(() -> new ClimberStopCommand(s_Climber)))
            .onFalse(new LeftClimberStopCommand(s_Climber));

        RightClimberUp.onTrue(new RightClimberUpCommand(s_Climber, s_Sensor).handleInterrupt(() -> new ClimberStopCommand(s_Climber)))
            .onFalse(new RightClimberStopCommand(s_Climber));
        RightClimberDown.onTrue(new RightClimberDownCommand(s_Climber).handleInterrupt(() -> new ClimberStopCommand(s_Climber)))
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
     * 
     */

     // have it return the autochooser selection
    public Command getAutonomousCommand() {
        // return new sourceSmartAuto(s_Sensor);
        // return autoChooser.getSelected();
        return AutoChooser.getSelected();
    }
}


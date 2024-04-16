package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.InfeedScoringCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.SuckBackCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LEDSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class InfeedShootCoCommand extends SequentialCommandGroup{

    
    public InfeedShootCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter, LEDSS s_LED) {

        addCommands(
                    new RepeatCommand(
                        new ConditionalCommand(
                            // while true
                                new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                        new InstantCommand(() -> s_LED.Blink()),
                                        new WristPIDCommand(s_Wrist, WristConstants.INFEED_SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
                                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                                        new SequentialCommandGroup(
                                            new WaitCommand(0.25),
                                            new SuckBackCoCommand(s_Infeed, s_Shooter)
                                        )
                                    ),
                                    new InstantCommand(() -> s_LED.Off()),
                                    new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                                    new WaitCommand(5)
                                ),

                            // while false 
                                new ParallelCommandGroup(
                                    new InfeedCoCommand(s_Wrist, s_Arm, s_Infeed),
                                    new ShooterCommand(s_Shooter, 0.0),
                                    new InstantCommand(() -> s_LED.Off())
                                    
                                )
                                .until(() -> s_Sensor.isTriggered()),

                            // condition
                            () -> s_Sensor.isTriggered()
                        )
                    )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
    }
    
    
} 
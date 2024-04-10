package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.SuckBackCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class AutoSourceInfeedCoCommand extends SequentialCommandGroup{

    
    public AutoSourceInfeedCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    // while true
                        new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                new WristPIDCommand(s_Wrist, WristConstants.DRIVE_POS, WristConstants.MAX_PID_OUTPUT),
                                new ArmPIDCommand(s_Arm, ArmConstants.DRIVE_POS, ArmConstants.MAX_PID_OUTPUT)),
                            new WaitCommand(0.25),
                            new InfeedCommand(s_Infeed, 0.0),
                            new WaitCommand(5)
                        ),
                        
                    // while false 
                        new ParallelCommandGroup(
                            new InfeedCoCommand(s_Wrist, s_Arm, s_Infeed),
                            new ShooterCommand(s_Shooter, 0.0)
                            
                        )
                        .until(() -> s_Sensor.isSensed()),
                        
                    // condition
                    () -> s_Sensor.isSensed()
                )
            )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
    
} 
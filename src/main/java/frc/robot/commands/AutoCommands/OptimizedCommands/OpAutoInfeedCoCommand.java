package frc.robot.commands.AutoCommands.OptimizedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class OpAutoInfeedCoCommand extends SequentialCommandGroup{

    
    public OpAutoInfeedCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new InfeedCommand(s_Infeed, 0.0),
                        new WaitCommand(5)
                    ),
                    
                    new ParallelCommandGroup(
                        new InfeedCommand(s_Infeed, InfeedConstants.INFEED_SPEED),
                        new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                        new WristPIDCommand(s_Wrist, WristConstants.INFEED_POS, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.INFEED_POS, WristConstants.MAX_PID_OUTPUT)
                    ),

                    () -> s_Sensor.isSensed()
                )
            )
        );
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
    }
    
    
} 
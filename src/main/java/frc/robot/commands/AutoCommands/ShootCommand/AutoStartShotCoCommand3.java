package frc.robot.commands.AutoCommands.ShootCommand;

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
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class AutoStartShotCoCommand3 extends SequentialCommandGroup{



    public AutoStartShotCoCommand3(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new ParallelCommandGroup(
                new WristPIDCommand(s_Wrist, WristConstants.START_POS_3, WristConstants.MAX_PID_OUTPUT),
                new ArmPIDCommand(s_Arm, ArmConstants.START_POS_3, ArmConstants.MAX_PID_OUTPUT),
                new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
            )  
        );

        addRequirements(s_Infeed, s_Shooter, s_Arm, s_Wrist);
    }
    
   
} 

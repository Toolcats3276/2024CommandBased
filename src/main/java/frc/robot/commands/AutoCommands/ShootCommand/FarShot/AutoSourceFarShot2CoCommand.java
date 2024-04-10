package frc.robot.commands.AutoCommands.ShootCommand.FarShot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.SuckBackCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class AutoSourceFarShot2CoCommand extends SequentialCommandGroup{



    public AutoSourceFarShot2CoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new ParallelCommandGroup(
                new WristPIDCommand(s_Wrist, WristConstants.SOURCE_FARSHOT_POS_2, WristConstants.MAX_PID_OUTPUT),
                new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                new SequentialCommandGroup(
                    new SuckBackCoCommand(s_Infeed, s_Shooter),
                    new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
                )
            )  
        );

        addRequirements(s_Infeed, s_Shooter, s_Arm, s_Wrist);
    }
    
   
} 

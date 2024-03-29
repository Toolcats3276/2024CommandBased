package frc.robot.commands.AutoCommands.OptimizedCommands.ShootCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class OpAutoRightShootCoCommands extends SequentialCommandGroup{



    public OpAutoRightShootCoCommands(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                    new ParallelCommandGroup(
                        new WristPIDCommand(s_Wrist, WristConstants.RIGHT_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT)
                    )
       
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
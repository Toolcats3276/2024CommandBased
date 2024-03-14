package frc.robot.commands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;

public class AutoAimCoCommand extends SequentialCommandGroup{

    
    public AutoAimCoCommand(WristSS s_Wrist, ArmSS s_Arm) {

        addCommands(
                    new ParallelCommandGroup(
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                        new AutoAimCommand(s_Wrist)
                    )
        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
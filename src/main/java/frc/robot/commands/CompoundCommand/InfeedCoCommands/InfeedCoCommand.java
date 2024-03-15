package frc.robot.commands.CompoundCommand.InfeedCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.InfeedCommand;
import frc.robot.commands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

import static frc.robot.Constants.WristConstants;

public class InfeedCoCommand extends SequentialCommandGroup{

    
    public InfeedCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed) {

        addCommands(
            new ParallelCommandGroup(
                new WristPIDCommand(s_Wrist, WristConstants.INFEED_POS, WristConstants.MAX_PID_OUTPUT),
                new ArmPIDCommand(s_Arm, ArmConstants.INFEED_POS, ArmConstants.MAX_PID_OUTPUT),
                new InfeedCommand(s_Infeed, InfeedConstants.INFEED_SPEED)
                )
        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
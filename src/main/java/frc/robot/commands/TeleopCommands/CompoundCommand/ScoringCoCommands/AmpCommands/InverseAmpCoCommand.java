package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

import static frc.robot.Constants.WristConstants;

public class InverseAmpCoCommand extends SequentialCommandGroup{

    
    public InverseAmpCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed) {

        addCommands(
            new ParallelCommandGroup(
                new ArmPIDCommand(s_Arm, ArmConstants.AMP_TWO_POS, ArmConstants.MAX_PID_OUTPUT),
                new WristPIDCommand(s_Wrist, WristConstants.AMP_TWO_POS, WristConstants.MAX_PID_OUTPUT),
                new InfeedCommand(s_Infeed, 0.0)
                )
        );
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
   
}
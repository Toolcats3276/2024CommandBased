package frc.robot.commands.CompoundCommand.ScoringCoCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class ToggleAmpCoCommand extends SequentialCommandGroup{

    
    public ToggleAmpCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed) {

        addCommands(
            new ConditionalCommand(
                // while true
                new ParallelCommandGroup(
                    new InverseAmpCoCommand(s_Wrist, s_Arm, s_Infeed)
                ),
                    
                // while false 
                new ParallelCommandGroup(
                    new AmpCoCommand(s_Wrist, s_Arm, s_Infeed)
                ),

                // condition
                () -> s_Arm.returnSetPoint() == ArmConstants.AMP_POSE
            )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
    
} 
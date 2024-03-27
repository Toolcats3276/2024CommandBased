package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class ToggleAmpCoCommand extends SequentialCommandGroup{

    
    public ToggleAmpCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter) {

        addCommands(
            new ConditionalCommand(
                // while true
                new ParallelCommandGroup(
                    new InverseAmpCoCommand(s_Wrist, s_Arm, s_Infeed),
                    new ShooterCommand(s_Shooter, 0.0)
                ),
                    
                // while false 
                new ParallelCommandGroup(
                    new AmpCoCommand(s_Wrist, s_Arm, s_Infeed),
                    new ShooterCommand(s_Shooter, 0.0)
                ),

                // condition
                () -> s_Arm.returnSetPoint() == ArmConstants.AMP_POSE
            )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
    
} 
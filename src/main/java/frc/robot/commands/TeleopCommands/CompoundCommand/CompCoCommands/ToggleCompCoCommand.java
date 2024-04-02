package frc.robot.commands.TeleopCommands.CompoundCommand.CompCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class ToggleCompCoCommand extends SequentialCommandGroup{

    
    public ToggleCompCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter, double armPIDOutput, double wristPIDOutput) {

        addCommands(
            new ConditionalCommand(
                // while true
                new ParallelCommandGroup(
                    new DriveCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter)
                ),
                    
                // while false 
                new ParallelCommandGroup(
                    new CompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter, armPIDOutput, wristPIDOutput)
                ),

                // condition
                () -> s_Arm.returnSetPoint() == ArmConstants.COMP_POS
            )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
    
} 
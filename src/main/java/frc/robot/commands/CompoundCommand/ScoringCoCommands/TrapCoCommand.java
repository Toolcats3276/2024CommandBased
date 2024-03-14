package frc.robot.commands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;

public class TrapCoCommand extends SequentialCommandGroup{

    
    public TrapCoCommand(WristSS s_Wrist, ArmSS s_Arm) {

        addCommands(

                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        // new WaitCommand(1),
                        // new WristPIDCommand(s_Wrist, WristConstants.CLIMBER_INIT_POS, WristConstants.SLOW_PID_OUTPUT),
                        new WaitCommand(1.5),
                        new WristPIDCommand(s_Wrist, WristConstants.CLIMBING_POS, WristConstants.SLOW_PID_OUTPUT),
                        new WaitCommand(0.5)
                    ),
                    new ArmPIDCommand(s_Arm, ArmConstants.CLIMBER_POS, ArmConstants.SLOW_PID_OUTPUT)
                )
        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
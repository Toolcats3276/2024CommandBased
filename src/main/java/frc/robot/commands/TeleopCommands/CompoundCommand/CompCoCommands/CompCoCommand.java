package frc.robot.commands.TeleopCommands.CompoundCommand.CompCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TeleopCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

import static frc.robot.Constants.WristConstants;

public class CompCoCommand extends SequentialCommandGroup{

    
    public CompCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter) {

        addCommands(
            new ParallelCommandGroup(
                new InfeedCommand(s_Infeed, 0.0),
                new ShooterCommand(s_Shooter, 0.0),
                new ArmPIDCommand(s_Arm, ArmConstants.COMP_POS, ArmConstants.MAX_PID_OUTPUT),
                new WristPIDCommand(s_Wrist, WristConstants.COMP_POS, WristConstants.MAX_PID_OUTPUT)
                )
        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
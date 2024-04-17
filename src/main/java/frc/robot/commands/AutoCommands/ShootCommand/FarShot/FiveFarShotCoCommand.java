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

public class FiveFarShotCoCommand extends SequentialCommandGroup{



    public FiveFarShotCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new SuckBackCoCommand(s_Infeed, s_Shooter),
            new ParallelCommandGroup(
                new WristPIDCommand(s_Wrist, WristConstants.MID_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
                new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                new ShooterCommand(s_Shooter, ShooterConstants.FAR_SHOT)
            )
        );

        addRequirements(s_Infeed, s_Shooter, s_Wrist, s_Arm);
    }
    
   
}



package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.ShooterSS;

public class SpeakerShotCoCommand extends SequentialCommandGroup{



    public SpeakerShotCoCommand(ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist, InfeedSS s_Infeed) {

        addCommands(
                // new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new WristPIDCommand(s_Wrist, WristConstants.SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                        new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                        new InfeedCommand(s_Infeed, 0.0)
                    )
        );

        addRequirements(s_Shooter, s_Arm, s_Wrist, s_Infeed);
    }
    
   
}
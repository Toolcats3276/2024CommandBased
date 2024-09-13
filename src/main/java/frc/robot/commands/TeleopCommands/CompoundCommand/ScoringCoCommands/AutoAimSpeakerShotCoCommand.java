package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.AutoAimCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.Swerve;

public class AutoAimSpeakerShotCoCommand extends SequentialCommandGroup{



    public AutoAimSpeakerShotCoCommand(ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist, InfeedSS s_Infeed, Swerve s_Swerve) {

        addCommands(
                // new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new AutoAimCommand(s_Wrist, s_Swerve),
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                        new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                        new InfeedCommand(s_Infeed, 0.0)
                    )
        );

        addRequirements(s_Shooter, s_Arm, s_Wrist, s_Infeed);
    }
    
   
}
package frc.robot.commands.AutoCommands.ShootCommand.AutoAimCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.AutoAimCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.AutoAimSwerve;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.Swerve;

public class AutoAutoAimCoCommand extends SequentialCommandGroup{



    public AutoAutoAimCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist, Swerve s_Swerve) {

        addCommands(
            new ParallelCommandGroup(
                new AutoAimCommand(s_Wrist, s_Swerve),
                new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                new ShooterCommand(s_Shooter, ShooterConstants.FAR_SHOT)
            )
        );

        addRequirements(s_Infeed, s_Shooter, s_Wrist, s_Arm);
    }
    
   
}



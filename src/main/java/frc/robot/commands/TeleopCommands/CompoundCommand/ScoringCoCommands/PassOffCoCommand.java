package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class PassOffCoCommand extends SequentialCommandGroup{



    public PassOffCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new WaitCommand(0),
            new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
            new WaitCommand(0.25),
            new ShooterCommand(s_Shooter, 0.0),
            new InfeedCommand(s_Infeed, 0.0)
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.SuckBackCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class TestShotCoCommand extends SequentialCommandGroup{


    public 
    TestShotCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new ConditionalCommand(

                new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist), 

                new SequentialCommandGroup(
                    new SuckBackCoCommand(s_Infeed, s_Shooter),
                    new ParallelCommandGroup(
                        new WristPIDCommand(s_Wrist, 0.512, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, 0.571, WristConstants.MAX_PID_OUTPUT),
                        new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
                    )
                ),
                
                () -> s_Arm.returnSetPoint() == 0.571)
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
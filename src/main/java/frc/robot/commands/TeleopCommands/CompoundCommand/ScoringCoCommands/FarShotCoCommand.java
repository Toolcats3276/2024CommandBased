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
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class FarShotCoCommand extends SequentialCommandGroup{


    public FarShotCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
                    new WaitCommand(0.25),
                    new InfeedCommand(s_Infeed, 0.0),
                    new ShooterCommand(s_Shooter, 0.0)
                ), 

                new ParallelCommandGroup(
                    new WristPIDCommand(s_Wrist, WristConstants.AUTO_FARSHOT_POS_2, WristConstants.MAX_PID_OUTPUT),
                    new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
                    new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
                ), 
                
                () -> s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS)
        );

        addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
    }
    
   
}
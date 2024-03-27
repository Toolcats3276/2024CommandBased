package frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands;

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

public class LeftNoteShootCoCommand extends SequentialCommandGroup{



    public LeftNoteShootCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new ParallelCommandGroup(
                new WristPIDCommand(s_Wrist, WristConstants.LEFT_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
                new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new ShooterCommand(s_Shooter, ShooterConstants.LEFT_SHOT),
                    new WaitCommand(ShooterConstants.SHOOT_DELAY),
                    new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
                    new WaitCommand(0.25),
                    new ShooterCommand(s_Shooter, 0.0),
                    new InfeedCommand(s_Infeed, 0.0)
                )
            ) 
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
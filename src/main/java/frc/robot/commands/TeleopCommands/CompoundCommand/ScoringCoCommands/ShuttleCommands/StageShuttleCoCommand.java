package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.PassOffCoCommand;
import frc.robot.commands.TeleopCommands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class StageShuttleCoCommand extends SequentialCommandGroup{

    public StageShuttleCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                new ConditionalCommand(
                    // on true
                    new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist),

                    // on false
                        new ParallelCommandGroup(
                            new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                            new WristPIDCommand(s_Wrist, WristConstants.SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
                            new ShooterCommand(s_Shooter, ShooterConstants.STAGE_SHUTTLE)
                    ),

                    // condition
                    () -> s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS)
            );
        addRequirements(s_Infeed, s_Shooter);
    }


    
   
}



package frc.robot.commands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.AutoCommands.ShootCommand.PassOffCoCommand;
import frc.robot.commands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class InverseScoreCommand extends SequentialCommandGroup{

    public InverseScoreCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                new ConditionalCommand(
                    // on true
                    new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist),

                    // on false
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ArmPIDCommand(s_Arm, ArmConstants.INVERSE_POS, ArmConstants.MAX_PID_OUTPUT),
                            new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
                        ),
                        new WaitCommand(0.75),
                            new WristPIDCommand(s_Wrist, WristConstants.INVERSE_POS, WristConstants.MAX_PID_OUTPUT)

                    ),

                    // condition
                    () -> s_Arm.returnSetPoint() == ArmConstants.INVERSE_POS)
            );
        addRequirements(s_Infeed, s_Shooter);
    }


    
   
}



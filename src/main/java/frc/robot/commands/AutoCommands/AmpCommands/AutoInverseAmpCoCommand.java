package frc.robot.commands.AutoCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.InfeedCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

import static frc.robot.Constants.WristConstants;

public class AutoInverseAmpCoCommand extends SequentialCommandGroup{

    
    public AutoInverseAmpCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter) {

        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new ArmPIDCommand(s_Arm, ArmConstants.AMP_TWO_POS, ArmConstants.MAX_PID_OUTPUT),
                    new WristPIDCommand(s_Wrist, WristConstants.AMP_TWO_POS, WristConstants.MAX_PID_OUTPUT),
                    new InfeedCommand(s_Infeed, 0.0)
                    ),
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new InfeedCommand(s_Infeed, InfeedConstants.AMP_TWO),
                    new ShooterCommand(s_Shooter, ShooterConstants.AMP_TWO)
                )
            )

        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
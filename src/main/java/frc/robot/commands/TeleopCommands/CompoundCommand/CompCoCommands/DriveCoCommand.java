package frc.robot.commands.TeleopCommands.CompoundCommand.CompCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

import static frc.robot.Constants.WristConstants;

public class DriveCoCommand extends SequentialCommandGroup{

    
    public DriveCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter) {

        addCommands(
            new ParallelCommandGroup(
                new InfeedCommand(s_Infeed, 0.0),
                new ShooterCommand(s_Shooter, 0.0),
                new ArmPIDCommand(s_Arm, ArmConstants.DRIVE_POS, ArmConstants.MAX_PID_OUTPUT),
                new WristPIDCommand(s_Wrist, WristConstants.DRIVE_POS, WristConstants.MAX_PID_OUTPUT)
                )
        );
        addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
    }
    
   
}
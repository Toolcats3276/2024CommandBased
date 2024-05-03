package frc.robot.commands.AutoCommands;

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
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class AutoPoofCoCommand extends SequentialCommandGroup{

    
    public AutoPoofCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter) {

        addCommands(
            // while true
                new ParallelCommandGroup(
                    new WristPIDCommand(s_Wrist, WristConstants.INFEED_POS, WristConstants.MAX_PID_OUTPUT),
                    new ArmPIDCommand(s_Arm, ArmConstants.INFEED_POS, ArmConstants.MAX_PID_OUTPUT),
                    new ShooterCommand(s_Shooter, ShooterConstants.POOF_SPEED),
                    new InfeedCommand(s_Infeed, InfeedConstants.POOF_SPEED)
                )
        );
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
} 
package frc.robot.commands.CompoundCommand;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.InfeedCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.z_ArmCommands.ArmManualCommand;
import frc.robot.commands.z_ClimberCommands.BothManualCommands.ClimberStopCommand;
import frc.robot.commands.z_WristCommands.WristManualCmd;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;

public class CancelCoCommand extends SequentialCommandGroup{



    public CancelCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter, ClimberSS s_Climber, SensorSS s_Sensor, DoubleSupplier WristAxis, DoubleSupplier ArmAxis) {

        addCommands(
            new ParallelCommandGroup(
                new WristManualCmd(s_Wrist, WristAxis),
                new ArmManualCommand(s_Arm, ArmAxis),
                new InfeedCommand(s_Infeed, 0.0),
                new ShooterCommand(s_Shooter, 0.0),
                new ClimberStopCommand(s_Climber)
                )
        );
        addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
    }
    
   
}
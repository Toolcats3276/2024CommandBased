package frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class SuckBackCoCommand extends SequentialCommandGroup{

    
    public SuckBackCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter) {

        addCommands(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        // new ShooterCommand(s_Shooter, -0.1), //kickback on shooter wheels maybe need negative
                        new InfeedCommand(s_Infeed, 0.1) //Kickback infeed
                    ), 
                    new WaitCommand(0.35), //delay
                    new ParallelCommandGroup(
                        new ShooterCommand(s_Shooter, 0.0), //stop motors
                        new InfeedCommand(s_Infeed, 0.0)
                    ),
                    new PrintCommand("suck back end")
                )
        );
        addRequirements(s_Infeed, s_Shooter);
    }   
    
   
}
package frc.robot.commands.CompoundCommand.InfeedCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.InfeedCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class SuckBackCommand extends SequentialCommandGroup{

    
    public SuckBackCommand(InfeedSS s_Infeed, ShooterSS s_Shooter) {

        addCommands(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new ShooterCommand(s_Shooter, -0.1), //kickback on shooter wheels maybe need negative
                        new InfeedCommand(s_Infeed, 0.1)), //Kickback infeed 
                    new WaitCommand(0.2), //delay
                    new ParallelCommandGroup(
                        new ShooterCommand(s_Shooter, 0.0), //stop motors
                        new InfeedCommand(s_Infeed, 0.0))
                )
        );
        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
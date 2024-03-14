package frc.robot.commands.CompoundCommand.ScoringCoCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ClimberCommands.BothManualCommands.ClimberStopCommand;
import frc.robot.commands.ClimberCommands.BothManualCommands.ClimberUpCommand2;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class AutoTrapCoCommand extends SequentialCommandGroup{

    
    public AutoTrapCoCommand(WristSS s_Wrist, ArmSS s_Arm, ClimberSS s_Climber) {

        addCommands(

                new ParallelCommandGroup(
                    new PathPlannerAuto("AutoClimb"),
                    new SequentialCommandGroup(
                        new WaitCommand(1.3),
                        new TrapCoCommand(s_Wrist, s_Arm),
                        new WaitCommand(1),
                        new ClimberUpCommand2(s_Climber),
                        new WaitCommand(0.5),
                        new ClimberStopCommand(s_Climber)
                    )
                )
        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;

public class AroundStageSmartAuto extends SequentialCommandGroup{

    
    public AroundStageSmartAuto(WristSS s_Wrists, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter, SensorSS s_Sensor) {
    
        addCommands(
          
            new SequentialCommandGroup(
                new PathPlannerAuto("source_0-X-1"),
                new ConditionalCommand(

                    new PathPlannerAuto("source_1-X-2"), 

                    new PathPlannerAuto("source_1-2"),
                    
                    () -> s_Sensor.isSensed()
                )
            )
        );
    }
}

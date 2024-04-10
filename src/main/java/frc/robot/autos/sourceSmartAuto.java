package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SensorSS;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class sourceSmartAuto extends SequentialCommandGroup{

    
    public sourceSmartAuto(SensorSS s_Sensor) {

        addCommands(
            new SequentialCommandGroup(
                new PathPlannerAuto("source_0-1"),

                new ConditionalCommand(

                    new PathPlannerAuto("source_1-2"),

                    new SequentialCommandGroup(
                        new WaitCommand(0.1),
                        new PathPlannerAuto("source_1-X-2")
                    ),

                    () -> !s_Sensor.isSensed()
                )
            )
        );
    }
}
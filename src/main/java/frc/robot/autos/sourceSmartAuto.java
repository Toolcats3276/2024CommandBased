package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SensorSS;

public class sourceSmartAuto extends SequentialCommandGroup{

    
    public sourceSmartAuto(SensorSS s_Sensor) {

        addCommands(
            new SequentialCommandGroup(
                /* first path */
                new PathPlannerAuto("source_0-1"),

                new ConditionalCommand(

                    /* if note is not picked up */
                    new SequentialCommandGroup(

                        new PathPlannerAuto("source_1-2"),

                        new ConditionalCommand(
                            /*if note is picked up */
                            new SequentialCommandGroup(
                                new PathPlannerAuto("source_2-X-3")
                            ),
                            /*if note is not picked up */
                            new SequentialCommandGroup(
                                new PathPlannerAuto("source_2-3")
                            ),
                            
                            () -> s_Sensor.isSensed()
                        )
                    ),

                    /* if note is picked up */
                    new SequentialCommandGroup(

                        new PathPlannerAuto("source_1-X-2"),

                        new ConditionalCommand(
                            /*if note is picked up */
                            new SequentialCommandGroup(
                                new PathPlannerAuto("source_2-X-3")
                            ),
                            /*if note is not picked up */
                            new SequentialCommandGroup(
                                new PathPlannerAuto("source_2-3")
                            ),
                            
                            () -> s_Sensor.isSensed())
                    ),

                    () -> !s_Sensor.isSensed()
                )
            )
        );
    }
}
package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.CompoundCommand.CompCoCommands.CompCoCommand;
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

//fist note 
                    /* picked up note */
                    new SequentialCommandGroup(
                        new PathPlannerAuto("source_1-X-2"), 
                        new ConditionalCommand(

        //second note
                            /* picked up note */
                            new PathPlannerAuto("around stage_2-X-3"),

        //second note
                            /* didn't pick up note */
                            new SequentialCommandGroup(
                                
                                new PathPlannerAuto("source_2-3"),
                                new ConditionalCommand(

        //third note
                                    /* picked up note */
                                    new PathPlannerAuto("around stage_3-x-2"), 
        //third note
                                    /* didn't pick up note */
                                    new CompCoCommand(s_Wrists, s_Arm, s_Infeed, s_Shooter, ArmConstants.MAX_PID_OUTPUT, WristConstants.MAX_PID_OUTPUT),
                                    () -> s_Sensor.isSensed()
                                )
                            ),
                            () -> s_Sensor.isSensed())
                    ),
//fist note 
                    /* didn't pick up note */
                    new SequentialCommandGroup(
                        new PathPlannerAuto("source_1-2"),
                        new ConditionalCommand(
                            new PathPlannerAuto("around stage_2-X-3"), 
                            new SequentialCommandGroup(
                                new PathPlannerAuto("source_2-3"),
                                new ConditionalCommand(
                                    new PathPlannerAuto("around stage_3-X"),
                                    new CompCoCommand(s_Wrists, s_Arm, s_Infeed, s_Shooter, ArmConstants.MAX_PID_OUTPUT, WristConstants.MAX_PID_OUTPUT),
                                    () -> s_Sensor.isSensed()  
                                )
                            ),
                            () -> s_Sensor.isSensed())
                    ),
                    () -> s_Sensor.isSensed()
                )
            )
        );
    }
}

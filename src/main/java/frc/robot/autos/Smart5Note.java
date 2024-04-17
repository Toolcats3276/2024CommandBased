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

public class Smart5Note extends SequentialCommandGroup{

    
    public Smart5Note(WristSS s_Wrists, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter, SensorSS s_Sensor) {
    
        addCommands(
          
            new SequentialCommandGroup(
                new PathPlannerAuto("StartFiveNote"),
                new ConditionalCommand(
                    new PathPlannerAuto("five_1-X-2"),
                    new SequentialCommandGroup(
                        new PathPlannerAuto("five_1-2")
                    ),
                    () -> s_Sensor.isSensed())
            
            )
        );
    }
}

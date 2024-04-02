package frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LEDSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;

public class ToggleInfeedCoCommand extends SequentialCommandGroup{
        
    public ToggleInfeedCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter, LEDSS s_LED) {

        addCommands(
            new ConditionalCommand(

                new ParallelCommandGroup(
                    new InfeedShootCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED),
                    new InstantCommand(() -> s_Sensor.setInfeedState(false))), 

                new ParallelCommandGroup(
                    new InfeedCompCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED),
                    new InstantCommand(() -> s_Sensor.setInfeedState(true))),

                () -> s_Sensor.getInfeedState())
        );
    }
}

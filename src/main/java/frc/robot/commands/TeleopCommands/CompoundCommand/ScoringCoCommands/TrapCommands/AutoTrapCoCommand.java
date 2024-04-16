// package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.commands.TeleopCommands.z_ClimberCommands.BothManualCommands.ClimberStopCommand;
// import frc.robot.commands.TeleopCommands.z_ClimberCommands.BothManualCommands.ClimberUpCommand2;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.ClimberSS;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.WristSS;

// public class AutoTrapCoCommand extends SequentialCommandGroup{

    
//     public AutoTrapCoCommand(WristSS s_Wrist, ArmSS s_Arm, ClimberSS s_Climber, Swerve s_Swerve) {

//         addCommands(
//             new SequentialCommandGroup(
//                 new InstantCommand(() -> s_Swerve.storeHeading()),
//                 new ParallelCommandGroup(
//                     new PathPlannerAuto("AutoClimb"),
//                     new SequentialCommandGroup(
//                         new WaitCommand(1.3),
//                         new TrapCoCommand(s_Wrist, s_Arm),
//                         new WaitCommand(1),
//                         new ClimberUpCommand2(s_Climber),
//                         new WaitCommand(0.5),
//                         new ClimberStopCommand(s_Climber)
//                     )
//                 ),
//                 new InstantCommand(() -> s_Swerve.setHeading(s_Swerve.getStoredHeading()))
//             ).handleInterrupt(() -> s_Swerve.setHeading(s_Swerve.getStoredHeading()))
//         );
//         addRequirements(s_Wrist, s_Arm);
//     }
    
   
// }

package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.TrapCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.BothManualCommands.ClimberUpCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSS;

public class AutoTrapCoCommand extends SequentialCommandGroup{

    
    public AutoTrapCoCommand(WristSS s_Wrist, ArmSS s_Arm, ClimberSS s_Climber, Swerve s_Swerve, SensorSS s_Sensor, InfeedSS s_Infeed) {

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.storeHeading()),
                new PathPlannerAuto("AutoTrap"),
                new WaitCommand(0.0),
                new ClimberUpCommand(s_Climber, s_Sensor),
                new InfeedCommand(s_Infeed, InfeedConstants.TRAP),
                new WaitCommand(1.75),
                new InfeedCommand(s_Infeed, 0.0)
                // new InstantCommand(() -> s_Swerve.setTrapHeading())
                )
            
        );
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
   
}
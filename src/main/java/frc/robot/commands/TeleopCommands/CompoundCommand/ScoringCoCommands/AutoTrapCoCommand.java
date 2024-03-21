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

package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.TeleopCommands.z_ClimberCommands.BothManualCommands.ClimberStopCommand;
import frc.robot.commands.TeleopCommands.z_ClimberCommands.BothManualCommands.ClimberUpCommand2;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSS;

public class AutoTrapCoCommand extends SequentialCommandGroup{

    
    public AutoTrapCoCommand(WristSS s_Wrist, ArmSS s_Arm, ClimberSS s_Climber, Swerve s_Swerve) {

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.storeHeading()),
                new PathPlannerAuto("AutoTrap")
                // new WaitCommand(1),
                // new ClimberUpCommand2(s_Climber),
                // new WaitCommand(2),
                // new ClimberStopCommand(s_Climber),
                // new InstantCommand(() -> s_Swerve.setTrapHeading())
                // ).handleInterrupt(() -> s_Swerve.setTrapHeading())
            )
        );
        addRequirements(s_Wrist, s_Arm);
    }
    
   
}
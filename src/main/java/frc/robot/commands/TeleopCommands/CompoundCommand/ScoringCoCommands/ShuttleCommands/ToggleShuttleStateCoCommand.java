// package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCoCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.LEDSS;
// import frc.robot.subsystems.SensorSS;
// import frc.robot.subsystems.ShooterSS;
// import frc.robot.subsystems.WristSS;

// public class ToggleShuttleStateCoCommand extends SequentialCommandGroup{

//     public ToggleShuttleStateCoCommand(SensorSS s_Sensor){

//         addCommands(
//             SequentialCommandGroup(
//                 new ConditionalCommand(
//                     new InstantCommand(() -> s_Sensor.setShuttleState(false)), 
//                     new InstantCommand(() -> s_Sensor.setShuttleState(true)), 
//                     () -> s_Sensor.getShuttleState()
//                 ),
//             new InstantCommand(() -> System.out.println(s_Sensor.getShuttleState()))
//             )
//         );

//     }
    
// }

package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SensorSS;


public class ToggleShuttleStateCoCommand extends SequentialCommandGroup{

    
    public ToggleShuttleStateCoCommand(SensorSS s_Sensor) {

        addCommands(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> s_Sensor.setShuttleState(false)), 
                    new InstantCommand(() -> s_Sensor.setShuttleState(true)), 
                    () -> s_Sensor.getShuttleState())
                    // new InstantCommand(() -> s_Sensor.setShuttleState(false))
            ).handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setShuttleState(false)))
        );   
    }
    
    
} 


// package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.WristConstants;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.LEDSS;
// import frc.robot.subsystems.SensorSS;
// import frc.robot.subsystems.ShooterSS;
// import frc.robot.subsystems.WristSS;


// public class ToggleShuttleCoCommand extends SequentialCommandGroup{

    
//     public ToggleShuttleCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, ShooterSS s_Shooter, SensorSS s_Sensor, LEDSS s_LED) {

//         addCommands(
//             new ConditionalCommand(
//                 // on true
//                 new ParallelCommandGroup(
//                     new ShuttleCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
//                 ),
                    
//                 // on false 
//                 new ParallelCommandGroup(
//                     new StageShuttleCoCommand(s_Wrist, s_Arm, s_Infeed, s_Sensor, s_Shooter, s_LED)
//                 ),

//                 // condition
//                 () -> s_Sensor.getShuttleState()
//             )
//         );
        
        
               
            
//         addRequirements(s_Wrist, s_Arm, s_Infeed);
//     }
    
    
// } 
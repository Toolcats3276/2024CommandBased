package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.SuckBackCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.PassOffCoCommand;
import frc.robot.commands.TeleopCommands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LEDSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class ShuttleCoCommand extends SequentialCommandGroup{

    
    public ShuttleCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter, LEDSS s_LED) {

        addCommands(
                    new RepeatCommand(
                        new ConditionalCommand(
                            // while true
                                new SequentialCommandGroup(
                                    
                                    new ParallelCommandGroup(
                                        new InstantCommand(() -> s_LED.Blink()),
                                        new WristPIDCommand(s_Wrist, WristConstants.SHUTTLE_POS, WristConstants.MAX_PID_OUTPUT),
                                        new ArmPIDCommand(s_Arm, ArmConstants.SHUTTLE_POS, ArmConstants.MAX_PID_OUTPUT)),
                                    new InstantCommand(() -> s_LED.Off()),
                                    new SuckBackCommand(s_Infeed, s_Shooter),
                                    new ShooterCommand(s_Shooter, ShooterConstants.SHUTTLE),
                                    new WaitCommand(ShooterConstants.SHOOT_DELAY),
                                    new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist)
                                ),

                            // while false 
                                new ParallelCommandGroup(
                                    new InfeedCoCommand(s_Wrist, s_Arm, s_Infeed),
                                    new ShooterCommand(s_Shooter, 0.0)
                                    
                                )
                                .until(() -> s_Sensor.isTriggered()),

                            // condition
                            () -> s_Sensor.isTriggered()
                        )
                    )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed);
    }
    
    
} 


// package frc.robot.commands.CompoundCommand.ScoringCoCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.InfeedConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.InfeedCommand;
// import frc.robot.commands.ShooterCommand;
// import frc.robot.commands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.WristSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.ShooterSS;

// public class ShuttleCoCommand extends SequentialCommandGroup{



//     public ShuttleCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

//         addCommands(
//                 new SequentialCommandGroup(
//                     new InfeedCommand(s_Infeed, 0.0),
//                     new ParallelCommandGroup(
//                         new InstantCommand(() -> System.out.println("speaker start")),
//                         new WristPIDCommand(s_Wrist, WristConstants.SHUTTLE_POS, WristConstants.SHUTTLE_PID_OUTPUT),
//                         new ArmPIDCommand(s_Arm, ArmConstants.SHUTTLE_POS, ArmConstants.MAX_PID_OUTPUT),
//                         new SequentialCommandGroup(
//                             new ShooterCommand(s_Shooter, ShooterConstants.SHUTTLE),
//                             new WaitCommand(ShooterConstants.SHOOT_DELAY),
//                             new WaitCommand(0.75),
//                             new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
//                             new WaitCommand(0.25),
//                             new ShooterCommand(s_Shooter, 0.0),
//                             new InfeedCommand(s_Infeed, 0.0),
//                             new InstantCommand(() -> System.out.println("speaker end")),
//                             new WaitCommand(5)

//                         )
//                     )
//                 )
//                 //     new WaitCommand(.5),
//                 //     new DriveCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter)
//                 // )  
//         );

//         addRequirements(s_Infeed, s_Shooter);
//     }
    
   
// }
package frc.robot.commands.AutoCommands.ShootCommand.FarShot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class AutoFarShotCoCommand extends SequentialCommandGroup{



    public AutoFarShotCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                // new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new WristPIDCommand(s_Wrist, WristConstants.AUTO_FARSHOT_POS, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                        new ShooterCommand(s_Shooter, ShooterConstants.FAR_SHOT)
                        
                    )
        );

        addRequirements(s_Infeed, s_Shooter, s_Wrist, s_Arm);
    }
    
   
}


// // package frc.robot.commands.AutoCommands;

// // import edu.wpi.first.wpilibj2.command.*;
// // import frc.robot.Constants.ArmConstants;
// // import frc.robot.Constants.InfeedConstants;
// // import frc.robot.Constants.ShooterConstants;
// // import frc.robot.Constants.WristConstants;
// // import frc.robot.commands.InfeedCommand;
// // import frc.robot.commands.ShooterCommand;
// // import frc.robot.commands.ArmCommands.ArmPIDCommand;
// // import frc.robot.commands.WristCommands.WristPIDCommand;
// // import frc.robot.subsystems.ArmSS;
// // import frc.robot.subsystems.InfeedSS;
// // import frc.robot.subsystems.SensorSS;
// // import frc.robot.subsystems.ShooterSS;
// // import frc.robot.subsystems.WristSS;


// // public class MidNoteShootCoCommand extends SequentialCommandGroup{

    
// //     public MidNoteShootCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter) {

// //         addCommands(
// //             new RepeatCommand(
// //                 new ConditionalCommand(
// //                     new SequentialCommandGroup(
// //                         new InfeedCommand(s_Infeed, 0.0),
// //                         new ParallelCommandGroup(
// //                             new WristPIDCommand(s_Wrist, WristConstants.MID_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
// //                             new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
// //                             new SequentialCommandGroup(
// //                                 new WaitCommand(0.7),
// //                                 new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
// //                                 new WaitCommand(0.1),
// //                                 new InfeedCommand(s_Infeed, 0.0),
// //                                 new WaitCommand(5)
// //                             )
// //                         )
// //                     ),
                    
// //                     new ParallelCommandGroup(
// //                         new InfeedCommand(s_Infeed, InfeedConstants.INFEED_SPEED),
// //                         new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
// //                         new WristPIDCommand(s_Wrist, WristConstants.INFEED_POS, WristConstants.MAX_PID_OUTPUT),
// //                         new ArmPIDCommand(s_Arm, ArmConstants.INFEED_POS, ArmConstants.MAX_PID_OUTPUT)
// //                     ),

// //                     () -> s_Sensor.isSensed()
// //                 )
// //             )
// //         );
        
               
            
// //         addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
// //     }
    
    
// // } 


// package frc.robot.commands.AutoCommands;

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

// public class AutoFarShotCoCommand extends SequentialCommandGroup{



//     public AutoFarShotCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

//         addCommands(
//                     new ParallelCommandGroup(
//                         new WristPIDCommand(s_Wrist, WristConstants.AUTO_FARSHOT_POS, WristConstants.MAX_PID_OUTPUT),
//                         new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
//                         new SequentialCommandGroup(
//                             new ShooterCommand(s_Shooter, ShooterConstants.FAR_SHOT),
//                             new WaitCommand(ShooterConstants.SHOOT_DELAY),
//                             new WaitCommand(0.15),
//                             new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
//                             new WaitCommand(0.25),
//                             new ShooterCommand(s_Shooter, 0.0),
//                             new InfeedCommand(s_Infeed, 0.0)

//                         )


//                     )
       
//         );

//         addRequirements(s_Infeed, s_Shooter);
//     }
    
   
// }
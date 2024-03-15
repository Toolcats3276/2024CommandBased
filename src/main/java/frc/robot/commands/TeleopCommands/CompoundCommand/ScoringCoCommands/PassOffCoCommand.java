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
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.SensorSS;
// import frc.robot.subsystems.ShooterSS;
// import frc.robot.subsystems.WristSS;


// public class RightNoteShootCoCommand extends SequentialCommandGroup{

    
//     public RightNoteShootCoCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter) {

//         addCommands(
//             new RepeatCommand(
//                 new ConditionalCommand(
//                     new SequentialCommandGroup(
//                         new InfeedCommand(s_Infeed, 0.0),
//                         new InstantCommand(() -> System.out.println("infeed stop")),
//                         new ParallelCommandGroup(
//                             new WristPIDCommand(s_Wrist, WristConstants.RIGHT_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
//                             new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
//                             new SequentialCommandGroup(
//                                 new InstantCommand(() -> System.out.println("start")),
//                                 new WaitCommand(0.7),
//                                 new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
//                                 new InstantCommand(() -> System.out.println("pass off")),
//                                 new WaitCommand(0.1),
//                                 new InfeedCommand(s_Infeed, 0.0),
//                                 new InstantCommand(() -> System.out.println("end")),
//                                 new WaitCommand(5)
                                
//                             )
//                         )
//                     ),

//                     new ParallelCommandGroup(
//                         new InfeedCommand(s_Infeed, InfeedConstants.INFEED_SPEED),
//                         new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
//                         new WristPIDCommand(s_Wrist, WristConstants.INFEED_POS, WristConstants.MAX_PID_OUTPUT),
//                         new ArmPIDCommand(s_Arm, ArmConstants.INFEED_POS, ArmConstants.MAX_PID_OUTPUT),
//                         new InstantCommand(() -> System.out.println("infeed"))
//                     ),
                    
                    
                
//                     () -> s_Sensor.isSensed()
//                 )
//             )
//         );
        
               
            
//         addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
//     }
    
    
// } 



package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.TeleopCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.ShooterCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class PassOffCoCommand extends SequentialCommandGroup{



    public PassOffCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new WaitCommand(0),
            new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
            new WaitCommand(0.25),
            new ShooterCommand(s_Shooter, 0.0),
            new InfeedCommand(s_Infeed, 0.0)
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
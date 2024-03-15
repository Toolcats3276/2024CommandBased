// package frc.robot.commands.AutoCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.ShooterCommand;
// import frc.robot.commands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.WristSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.ShooterSS;

// public class RightNoteShootCoCommand extends SequentialCommandGroup{



//     public RightNoteShootCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

//         addCommands(
//                 // new SequentialCommandGroup(
//                     new ParallelCommandGroup(
//                         new WristPIDCommand(s_Wrist, WristConstants.RIGHT_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
//                         new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
//                         new SequentialCommandGroup(
//                             new WaitCommand(0.25),
//                             new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
//                         )


//                     )
//                 //     new WaitCommand(.5),
//                 //     new DriveCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter)
//                 // )  
//         );

//         addRequirements(s_Infeed, s_Shooter, s_Wrist, s_Arm);
//     }
    
   
// }



package frc.robot.commands.AutoCommands.ShootCommand.CloseNoteCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class RightNoteShootCoCommand extends SequentialCommandGroup{



    public RightNoteShootCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                // new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new WristPIDCommand(s_Wrist, WristConstants.RIGHT_NOTE_POS, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                        new SequentialCommandGroup(
                            new WaitCommand(0.25),
                            new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                            new WaitCommand(ShooterConstants.SHOOT_DELAY),
                            new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
                            new WaitCommand(0.25),
                            new ShooterCommand(s_Shooter, 0.0),
                            new InfeedCommand(s_Infeed, 0.0)

                        )


                    )
                //     new WaitCommand(.5),
                //     new DriveCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter)
                // )  
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
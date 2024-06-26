// package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
// import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.WristSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.ShooterSS;

// public class InverseScoreCommand extends SequentialCommandGroup{

//     public InverseScoreCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

//         addCommands(
//                 new ConditionalCommand(
//                     // on true
//                     new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist),

//                     // on false
//                     new SequentialCommandGroup(
//                         new ParallelCommandGroup(
//                             new ArmPIDCommand(s_Arm, ArmConstants.INVERSE_POS, ArmConstants.MAX_PID_OUTPUT),
//                             new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
//                         ),
//                         new WaitCommand(0.75),
//                             new WristPIDCommand(s_Wrist, WristConstants.INVERSE_POS, WristConstants.MAX_PID_OUTPUT)

//                     ),

//                     // condition
//                     () -> s_Arm.returnSetPoint() == ArmConstants.INVERSE_POS)
//             );
//         addRequirements(s_Infeed, s_Shooter);
//     }


    
   
// }


package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.InfeedScoringCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.InfeedCoCommand;
import frc.robot.commands.TeleopCommands.CompoundCommand.InfeedCoCommands.SuckBackCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LEDSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;
import frc.robot.subsystems.WristSS;


public class InverseScoreCommand extends SequentialCommandGroup{

    
    public InverseScoreCommand(WristSS s_Wrist, ArmSS s_Arm, InfeedSS s_Infeed, SensorSS s_Sensor, ShooterSS s_Shooter, LEDSS s_LED) {

        addCommands(
                    new RepeatCommand(
                        new ConditionalCommand(
                            // while true
                            new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                    new ArmPIDCommand(s_Arm, ArmConstants.INVERSE_POS, ArmConstants.MAX_PID_OUTPUT),
                                    new InstantCommand(() -> s_LED.Blink()),
                                    new SequentialCommandGroup(
                                        new WaitCommand(0.25),
                                        new SuckBackCoCommand(s_Infeed, s_Shooter),
                                        new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)
                                    ),
                                    new SequentialCommandGroup( 
                                        new WaitCommand(0.2),
                                        new ParallelCommandGroup(
                                            new WristPIDCommand(s_Wrist, WristConstants.INVERSE_POS, WristConstants.MAX_PID_OUTPUT),
                                            new InstantCommand(() -> s_LED.Off())
                                        )
                                    ) 
                                ),
                                new WaitCommand(5)
                            ),

                            // while false 
                                new ParallelCommandGroup(
                                    new InfeedCoCommand(s_Wrist, s_Arm, s_Infeed),
                                    new ShooterCommand(s_Shooter, 0.0),
                                    new InstantCommand(() -> s_LED.Off())
                                )
                                .until(() -> s_Sensor.isTriggered()),

                            // condition
                            () -> s_Sensor.isTriggered()
                        )
                    )
        );
        
        
               
            
        addRequirements(s_Wrist, s_Arm, s_Infeed, s_Shooter);
    }
    
    
} 
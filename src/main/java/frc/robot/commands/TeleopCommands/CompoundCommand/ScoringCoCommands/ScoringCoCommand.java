package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.ShooterSS;

public class ScoringCoCommand extends SequentialCommandGroup{

    public ScoringCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist, SensorSS s_Sensor) {

        addCommands(
                new ConditionalCommand(

                    // ON TRUE     conditional command for speaker shot and shuttle shot
                    new ConditionalCommand(

                        new SpeakerShotCoCommand(s_Shooter, s_Arm, s_Wrist), 

                        new ParallelCommandGroup(
                            new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist),
                            new InstantCommand(() -> s_Sensor.setShuttleState(false))
                        ),

                        () -> s_Arm.returnSetPoint() == ArmConstants.COMP_POS),


                    // ON FALSE     scoring command for amp and inverse amp
                    new ConditionalCommand(

                        /* on true      scoring command for inverse amp */
                        new ParallelCommandGroup(
                            new InfeedCommand(s_Infeed, InfeedConstants.AMP_TWO),
                            new ShooterCommand(s_Shooter, ShooterConstants.AMP_TWO)),
                        
                        /* on false     scoring command for amp */
                        new InfeedCommand(s_Infeed, InfeedConstants.AMP),

                        /* condition */
                        () -> s_Arm.returnSetPoint() == ArmConstants.AMP_TWO_POS),
                    
                    // CONDITION
                    () -> s_Arm.returnSetPoint() == ArmConstants.INFEED_POS ||
                    s_Arm.returnSetPoint() == ArmConstants.DRIVE_POS ||
                    s_Arm.returnSetPoint() == ArmConstants.COMP_POS ||
                    s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS ||
                    s_Arm.returnSetPoint() == ArmConstants.SHUTTLE_POS || 
                    s_Arm.returnSetPoint() == ArmConstants.STAGE_SHUTTLE_POS ||
                    s_Arm.returnSetPoint() == ArmConstants.INVERSE_POS)
        );
        addRequirements(s_Infeed, s_Shooter);
    }


    
   
}



// package frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.InfeedConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.TeleopCommands.BaseCommands.InfeedCommand;
// import frc.robot.commands.TeleopCommands.BaseCommands.ShooterCommand;
// import frc.robot.commands.TeleopCommands.BaseCommands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.TeleopCommands.BaseCommands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.WristSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.SensorSS;
// import frc.robot.subsystems.ShooterSS;

// public class ScoringCoCommand extends SequentialCommandGroup{

//     public ScoringCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist, SensorSS s_Sensor) {

//         addCommands(
//                 new ConditionalCommand(

//                     // ON TRUE     conditional command for speaker shot and shuttle shot
//                         new ConditionalCommand(

//                             /* on true      condition command for shuttle shots, sets the shuttle state to false */
//                             new ParallelCommandGroup(
//                                 new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist),
//                                 new InstantCommand(() -> s_Sensor.setShuttleState(false))),
                            
//                             /* on false     conditional command for speaker shot */
//                             new ConditionalCommand(

//                                 // ON TRUE      command to shoot speaker shot
//                                 new PassOffCoCommand(s_Infeed, s_Shooter, s_Arm, s_Wrist),

//                                 // ON FALSE     command to move arm and wrist to speaker shot position and ramp motors
//                                 new ParallelCommandGroup(
//                                     new WristPIDCommand(s_Wrist, WristConstants.SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
//                                     new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
//                                     new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER)),
                            
//                                 // CONDITION
//                                 () -> s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS && s_Wrist.returnSetPoint() == WristConstants.SPEAKER_POS),
                            
//                             /* condition */
//                             () -> s_Arm.returnSetPoint() == ArmConstants.SHUTTLE_POS ||
//                             s_Wrist.returnSetPoint() == WristConstants.STAGE_SHUTTLE_POS
                        
//                     ), 


//                 // ON FALSE     scoring command for amp and inverse amp
//                     new ConditionalCommand(

//                         /* on true      scoring command for inverse amp */
//                         new ParallelCommandGroup(
//                             new InfeedCommand(s_Infeed, InfeedConstants.AMP_TWO),
//                             new ShooterCommand(s_Shooter, ShooterConstants.AMP_TWO)),
                        
//                         /* on false     scoring command for amp */
//                         new InfeedCommand(s_Infeed, InfeedConstants.AMP),

//                         /* condition */
//                         () -> s_Arm.returnSetPoint() == ArmConstants.AMP_TWO_POS),
                    
//                 // CONDITION
//                     () -> s_Arm.returnSetPoint() == ArmConstants.INFEED_POS ||
//                     s_Arm.returnSetPoint() == ArmConstants.DRIVE_POS ||
//                     s_Arm.returnSetPoint() == ArmConstants.COMP_POS ||
//                     s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS ||
//                     s_Arm.returnSetPoint() == ArmConstants.SHUTTLE_POS || 
//                     s_Wrist.returnSetPoint() == WristConstants.STAGE_SHUTTLE_POS)
//         );
//         addRequirements(s_Infeed, s_Shooter);
//     }


    
   
// }

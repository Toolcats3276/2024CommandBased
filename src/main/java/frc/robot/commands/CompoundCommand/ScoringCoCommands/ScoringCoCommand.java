package frc.robot.commands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.InfeedCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.z_ArmCommands.ArmPIDCommand;
import frc.robot.commands.z_WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class ScoringCoCommand extends SequentialCommandGroup{

    public ScoringCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                new ConditionalCommand(

                // on true
                    new ConditionalCommand(
                        //on true
                        new SequentialCommandGroup(
                            new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
                            new WaitCommand(0.25),
                            new ShooterCommand(s_Shooter, 0.0),
                            new InfeedCommand(s_Infeed, 0.0),
                            new InstantCommand(() -> System.out.println("false")),
                            new WaitCommand(5)
                        ),
                        //on false
                        new ParallelCommandGroup(
                            new WristPIDCommand(s_Wrist, WristConstants.SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
                            new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                            new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                            new InstantCommand(() -> System.out.println("true"))),
                            
                        //condition
                        () -> s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS
                    ), 

                // on false
                    new ConditionalCommand(
                        //on true
                        new ParallelCommandGroup(
                            new InfeedCommand(s_Infeed, InfeedConstants.AMP_TWO),
                            new ShooterCommand(s_Shooter, ShooterConstants.AMP_TWO)),
                        
                        //on false
                        new InfeedCommand(s_Infeed, InfeedConstants.AMP),

                        // condition
                        () -> s_Arm.returnSetPoint() == ArmConstants.AMP_TWO_POS),
                    
                // condition
                    () -> s_Arm.returnSetPoint() == ArmConstants.DRIVE_POS || s_Arm.returnSetPoint() == ArmConstants.COMP_POS || s_Arm.returnSetPoint() == ArmConstants.SPEAKER_POS)
        );
        addRequirements(s_Infeed, s_Shooter);
    }


    
   
}


/*package frc.robot.commands.CompoundCommand.ScoringCoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.InfeedCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.ShooterSS;

public class ScoringCoCommand extends SequentialCommandGroup{

    public ScoringCoCommand(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        new WristPIDCommand(s_Wrist, WristConstants.SPEAKER_POS, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.SPEAKER_POS, ArmConstants.MAX_PID_OUTPUT),
                        // new AutoAimCoCommand(s_Wrist, s_Arm),
                        new SequentialCommandGroup(
                            new ShooterCommand(s_Shooter, ShooterConstants.SPEAKER),
                            new WaitCommand(ShooterConstants.SHOOT_DELAY),
                            new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
                            new WaitCommand(0.25),
                            new ShooterCommand(s_Shooter, 0.0),
                            new InfeedCommand(s_Infeed, 0.0),
                            new WaitCommand(5)
                    )), 
                    new ConditionalCommand(
                        new ParallelCommandGroup(
                            new InfeedCommand(s_Infeed, InfeedConstants.AMP_TWO),
                            new ShooterCommand(s_Shooter, ShooterConstants.AMP_TWO)),
                        new InfeedCommand(s_Infeed, InfeedConstants.AMP),
                        () -> s_Arm.returnSetPoint() == ArmConstants.AMP_TWO_POS),
                    
                    () -> s_Arm.returnSetPoint() == ArmConstants.INFEED_POS || s_Arm.returnSetPoint() == ArmConstants.DRIVE_POS || s_Arm.returnSetPoint() == ArmConstants.COMP_POS)
            )
        );
        addRequirements(s_Infeed, s_Shooter);
    }


    
   
} */
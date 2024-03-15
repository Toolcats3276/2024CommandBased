package frc.robot.commands.AutoCommands.ShootCommand;

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

public class AutoStartShotCoCommand2 extends SequentialCommandGroup{



    public AutoStartShotCoCommand2(InfeedSS s_Infeed, ShooterSS s_Shooter, ArmSS s_Arm, WristSS s_Wrist) {

        addCommands(
                // new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> System.out.println("speaker start")),
                        new WristPIDCommand(s_Wrist, WristConstants.START_POS_2, WristConstants.MAX_PID_OUTPUT),
                        new ArmPIDCommand(s_Arm, ArmConstants.START_POS_2, ArmConstants.MAX_PID_OUTPUT),
                        new SequentialCommandGroup(
                            new ShooterCommand(s_Shooter, ShooterConstants.START_SHOT),
                            new WaitCommand(ShooterConstants.SHOOT_DELAY),
                            new WaitCommand(0.25),
                            new InfeedCommand(s_Infeed, InfeedConstants.PASS_OFF),
                            new WaitCommand(0.15),
                            new ShooterCommand(s_Shooter, 0.0),
                            new InfeedCommand(s_Infeed, 0.0),
                            new InstantCommand(() -> System.out.println("speaker end")),
                            new WaitCommand(5)

                        )


                    )
                //     new WaitCommand(.5),
                //     new DriveCoCommand(s_Wrist, s_Arm, s_Infeed, s_Shooter)
                // )  
        );

        addRequirements(s_Infeed, s_Shooter);
    }
    
   
}
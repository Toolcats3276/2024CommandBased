package frc.robot.commands.z_ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSS;

public class ArmFeedForwardCommand extends Command {
    
    private ArmSS s_Arm;



    public ArmFeedForwardCommand(ArmSS s_Arm) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);
    }

    @Override
    public void initialize() {
      s_Arm.ManualFeedForward();

    }

    @Override
    public void execute() {

        
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

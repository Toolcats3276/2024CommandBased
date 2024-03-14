package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSS;

public class ArmStopCommand extends Command {
    
    private ArmSS s_Arm;



    public ArmStopCommand(ArmSS s_Arm) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);
    }

    @Override
    public void initialize() {
      s_Arm.ManualStop();

    }

    @Override
    public void execute() {
        System.out.println("stopped");
        
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

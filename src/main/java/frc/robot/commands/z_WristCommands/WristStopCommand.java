package frc.robot.commands.z_WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class WristStopCommand extends Command {
    
    private WristSS s_Wrist;



    public WristStopCommand(WristSS s_Wrist) {
        this.s_Wrist = s_Wrist;
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
      s_Wrist.ManualStop();

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

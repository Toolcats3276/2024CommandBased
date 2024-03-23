package frc.robot.commands.TeleopCommands.BaseCommands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class WristUpCommand extends Command {
    
    private WristSS s_Wrist;



    public WristUpCommand(WristSS s_Wrist) {
        this.s_Wrist = s_Wrist;
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
      s_Wrist.ManualUp();

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

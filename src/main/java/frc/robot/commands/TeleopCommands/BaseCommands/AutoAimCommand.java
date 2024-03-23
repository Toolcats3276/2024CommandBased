package frc.robot.commands.TeleopCommands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class AutoAimCommand extends Command {
    
    private WristSS s_Wrist;



    public AutoAimCommand(WristSS s_Wrist) {
        this.s_Wrist = s_Wrist;

        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
      s_Wrist.AutoAim();
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

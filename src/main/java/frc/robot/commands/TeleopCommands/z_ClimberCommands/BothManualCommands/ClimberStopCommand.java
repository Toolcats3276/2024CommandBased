package frc.robot.commands.TeleopCommands.z_ClimberCommands.BothManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;

public class ClimberStopCommand extends Command {
    
    private ClimberSS s_Climber;



    public ClimberStopCommand(ClimberSS s_Climber) {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {
      s_Climber.ManualStop();

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

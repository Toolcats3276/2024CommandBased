package frc.robot.commands.ClimberCommands.LeftManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;

public class LeftClimberDownCommand extends Command {
    
    private ClimberSS s_Climber;



    public LeftClimberDownCommand(ClimberSS s_Climber) {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
    
    s_Climber.LeftDown();
        
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

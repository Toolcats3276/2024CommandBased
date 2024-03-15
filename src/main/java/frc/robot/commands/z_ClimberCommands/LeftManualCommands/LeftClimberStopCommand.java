package frc.robot.commands.z_ClimberCommands.LeftManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;

public class LeftClimberStopCommand extends Command {
    
    private ClimberSS s_Climber;



    public LeftClimberStopCommand(ClimberSS s_Climber) {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
    System.out.println("leftstop");
    s_Climber.LeftStop();
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

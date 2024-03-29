package frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.BothManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;

public class ClimberUpCommand2 extends Command {
    
    private ClimberSS s_Climber;


    public ClimberUpCommand2(ClimberSS s_Climber) {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        s_Climber.ManualUp();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

package frc.robot.commands.TeleopCommands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSS;

public class AutoAimCommand extends Command {
    
    private WristSS s_Wrist;
    private Swerve s_Swerve;



    public AutoAimCommand(WristSS s_Wrist, Swerve s_Swerve) {
        this.s_Wrist = s_Wrist;
        this.s_Swerve = s_Swerve;

        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
    s_Swerve.setAutoAimState(true);


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

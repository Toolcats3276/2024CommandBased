package frc.robot.commands.TeleopCommands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfeedSS;

public class InfeedCommand extends Command {
    
    private InfeedSS s_Infeed;
    private double speed;


    public InfeedCommand(InfeedSS s_Infeed, Double speed) {
        this.s_Infeed = s_Infeed;
        this.speed = speed;
        addRequirements(s_Infeed);
    }

    @Override
    public void initialize() {
        s_Infeed.setSpeed(speed);
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
    


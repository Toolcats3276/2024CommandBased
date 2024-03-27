package frc.robot.commands.TeleopCommands.BaseCommands.ClimberCommands.LeftManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.SensorSS;

public class LeftClimberUpCommand extends Command {
    
    private ClimberSS s_Climber;
    private SensorSS s_Sensor;



    public LeftClimberUpCommand(ClimberSS s_Climber, SensorSS s_Sensor) {
        this.s_Climber = s_Climber;
        this.s_Sensor = s_Sensor;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
        if(s_Sensor.returnLeftLimit()){
            s_Climber.LeftStop();
        }
        else{
            s_Climber.LeftUp();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

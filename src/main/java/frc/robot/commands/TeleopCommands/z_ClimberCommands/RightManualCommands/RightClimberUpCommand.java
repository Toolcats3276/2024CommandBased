package frc.robot.commands.TeleopCommands.z_ClimberCommands.RightManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.SensorSS;

public class RightClimberUpCommand extends Command {
    
    private ClimberSS s_Climber;
    private SensorSS s_Sensor;



    public RightClimberUpCommand(ClimberSS s_Climber, SensorSS s_Sensor) {
        this.s_Climber = s_Climber;
        this.s_Sensor = s_Sensor;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(s_Sensor.returnRightLimit()){
            s_Climber.RightStop();
        }
        else{
            s_Climber.RightUp();
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

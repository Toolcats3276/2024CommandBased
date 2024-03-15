package frc.robot.commands.TeleopCommands.z_ClimberCommands.BothManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.SensorSS;

public class ClimberUpCommand extends Command {
    
    private ClimberSS s_Climber;
    private SensorSS s_Sensor;



    public ClimberUpCommand(ClimberSS s_Climber, SensorSS s_Sensor) {
        this.s_Climber = s_Climber;
        this.s_Sensor = s_Sensor;
        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(s_Sensor.returnLeftLimit() || s_Sensor.returnRightLimit()){
            s_Climber.ManualStop();
        }
        else{
            s_Climber.ManualUp();
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        s_Climber.ManualStop();
    }

    @Override
    public boolean isFinished() {
        return s_Sensor.returnLeftLimit() || s_Sensor.returnRightLimit();
    }
    
}

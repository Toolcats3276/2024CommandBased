package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopCommands.CompoundCommand.ScoringCoCommands.ShuttleCommands.ShuttleCoCommand;

/* SEE WristSS FOR EXPLANATIONS */

public class SensorSS extends SubsystemBase{

    /* CREATS DEBOUNCER OBJECTS FOR SENSOR INPUT
     */
    private final DigitalInput sensor;

    private final DigitalInput LLimitSwitch;
    private final DigitalInput RLimitSwitch;
    
    private final Debouncer m_triggerDebouncer;
    private final Debouncer m_infeedDebouncer;
    private final Debouncer m_infeedShotDebouncer;
    private final Debouncer m_inverseDebouncer;
    private final Debouncer m_shuttleDebouncer;

    private boolean ShuttleState;
    private boolean InfeedState = false;

   public SensorSS() {

    sensor = new DigitalInput(0);

    LLimitSwitch = new DigitalInput(1);
    RLimitSwitch = new DigitalInput(9);
    
    m_triggerDebouncer = new Debouncer(0.1, DebounceType.kBoth);
    m_infeedDebouncer = new Debouncer(0.85, DebounceType.kBoth); //0.67
    m_infeedShotDebouncer = new Debouncer(0.85, DebounceType.kBoth);
    m_inverseDebouncer = new Debouncer(2.5, DebounceType.kBoth);
    m_shuttleDebouncer = new Debouncer(2.5, DebounceType.kBoth);

   }


    public void periodic(){
        SmartDashboard.putBoolean("Sensor", sensor.get());
        SmartDashboard.putBoolean("LeftLimit", !LLimitSwitch.get());
        SmartDashboard.putBoolean("RightLimit", !RLimitSwitch.get());
        if(ShuttleState){
            SmartDashboard.putString("ShuttleState", "Over Stage");
        }
        else if(!ShuttleState){
            SmartDashboard.putString("ShuttleState", "Under Stage");
        }

    }
    
    public boolean isSensed(){
        return sensor.get();
    }
    
    /* returns weather or not sensor sees something with a small debounce
     * should be used for conditions on conditional commands
     */
    public boolean isTriggered(){
        return m_triggerDebouncer.calculate(sensor.get());
    }

    /* returns weather or not sensor sees something with a larger debounce
     * should be used for ending infeed repeat commands
    */
    public boolean infeedDelay(){
        return m_infeedDebouncer.calculate(sensor.get());
    }

    public boolean infeedShotDelay(){
        return m_infeedShotDebouncer.calculate(sensor.get());
    }

    public boolean inverseDelay(){
        return m_inverseDebouncer.calculate(sensor.get());
    }

    public boolean shuttleDelay(){
        return m_shuttleDebouncer.calculate(sensor.get());
    }



    public boolean returnLeftLimit(){
        return !LLimitSwitch.get();
    }

    public boolean returnRightLimit(){
        return !RLimitSwitch.get();
    }
    
    public void setShuttleState(boolean ShuttleState){
        this.ShuttleState = ShuttleState;
    }

    public boolean getShuttleState(){
        return ShuttleState;
    }

    public void setInfeedState(boolean InfeedState){
        this.InfeedState = InfeedState;
    }

    public boolean getInfeedState(){
        return InfeedState;
    }




    





    
}

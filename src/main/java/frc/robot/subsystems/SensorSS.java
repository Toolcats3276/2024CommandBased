package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* SEE WristSS FOR EXPLANATIONS */

public class SensorSS extends SubsystemBase{

    /* CREATS DEBOUNCER OBJECTS FOR SENSOR INPUT
     */

    private final DigitalInput sensor;

    private final DigitalInput LLimitSwitch;
    private final DigitalInput RLimitSwitch;
    
    private final Debouncer m_debouncer1;
    private final Debouncer m_debouncer2;

   public SensorSS() {

    sensor = new DigitalInput(0);

    LLimitSwitch = new DigitalInput(1);
    RLimitSwitch = new DigitalInput(9);
    
    m_debouncer1 = new Debouncer(0.1, DebounceType.kBoth);
    m_debouncer2 = new Debouncer(0.65, DebounceType.kBoth);

   }


    public void periodic(){
        SmartDashboard.putBoolean("Sensor", sensor.get());
        SmartDashboard.putBoolean("LeftLimit", !LLimitSwitch.get());
        SmartDashboard.putBoolean("RightLimit", !RLimitSwitch.get());

    }
    
    /* returns weather or not sensor sees something with a small debounce
     * should be used for conditions on conditional commands
     */
    public boolean isTriggered(){
        return m_debouncer1.calculate(sensor.get());
    }

    /* returns weather or not sensor sees something with a larger debounce
     * should be used for ending infeed repeat commands
    */
    public boolean infeedDelay(){
        return m_debouncer2.calculate(sensor.get());
    }


    public boolean returnLeftLimit(){
        return !LLimitSwitch.get();
    }

    public boolean returnRightLimit(){
        return !RLimitSwitch.get();
    }

    public boolean isSensed(){
        return sensor.get();
    }

    





    
}

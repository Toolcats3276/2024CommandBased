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
    private final DigitalInput LLimitSwitch;
    private final DigitalInput RLimitSwitch;
    private final DigitalInput sensor;
    private final Debouncer m_debouncer;
    private final Debouncer m_debouncer2;
    private final Debouncer m_debouncer3;
    private final Debouncer m_debouncer4;

   public SensorSS() {
    LLimitSwitch = new DigitalInput(1);
    RLimitSwitch = new DigitalInput(9);
    

    sensor = new DigitalInput(0);
    
    m_debouncer = new Debouncer(0.4, DebounceType.kBoth);
    m_debouncer2 = new Debouncer(0.65, DebounceType.kBoth); //2 seconds
    m_debouncer3 = new Debouncer(0.1, DebounceType.kBoth);
    m_debouncer4 = new Debouncer(3, DebounceType.kBoth);
   }


    public void periodic(){
        SmartDashboard.putBoolean("Sensor", sensor.get());
        SmartDashboard.putBoolean("LeftLimit", !LLimitSwitch.get());
        SmartDashboard.putBoolean("RightLimit", !RLimitSwitch.get());

    }
    
    public boolean isTriggered(){
        return m_debouncer3.calculate(sensor.get());
    }

    public boolean shootDelay(){
        if (!m_debouncer.calculate(sensor.get())){
            System.out.println("sensor shoot end");
        }
        return m_debouncer.calculate(sensor.get());
    }
    public boolean autoInfeedDelay(){

        return m_debouncer4.calculate(sensor.get()) && !sensor.get();
    }

    public boolean isDebounced(){
        if (m_debouncer2.calculate(sensor.get())){
            System.out.println("sensor debounced");
        }
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

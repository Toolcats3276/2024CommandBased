package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class InfeedSS extends SubsystemBase {


    private TalonFX m_infeedLeadMotor;
    private TalonFX m_infeedFollowMotor;
    private double speed;
    private Timer infeedTimer;

  
    public InfeedSS(){
            m_infeedLeadMotor = new TalonFX(InfeedConstants.INFEED_LEAD_MOTOR_ID);
            m_infeedLeadMotor.getConfigurator().apply(new TalonFXConfiguration());
            m_infeedLeadMotor.setNeutralMode(NeutralModeValue.Brake);
            m_infeedLeadMotor.setInverted(false);

            m_infeedFollowMotor = new TalonFX(InfeedConstants.INFEED_FOLLOW_MOTOR_ID);
            m_infeedFollowMotor.getConfigurator().apply(new TalonFXConfiguration());
            m_infeedFollowMotor.setNeutralMode(NeutralModeValue.Brake);
            m_infeedFollowMotor.setInverted(false);
            m_infeedFollowMotor.setControl(new StrictFollower(m_infeedLeadMotor.getDeviceID()));

            infeedTimer = new Timer();
    }


    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode InfeedMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(InfeedMode) {

            case Stop:{
                m_infeedLeadMotor.set(0);
                break;
            }

            case SetSpeed:{
                m_infeedLeadMotor.set(speed);
                break;
            }
        }

        SmartDashboard.putNumber("InfeedSpeed", speed);
    }



    public void Stop(){
        InfeedMode = Mode.Stop;
    }


    
    public void setSpeed(double speed){
        this.speed = speed;
        InfeedMode = Mode.SetSpeed;
    }



    public void timerReset(){
        infeedTimer.reset();
    }



    public void timerSpeed(double speed){
        this.speed = speed;
        infeedTimer.start();
        if(infeedTimer.hasElapsed(1)){

            InfeedMode = Mode.SetSpeed;
        }
    }

    
    
}
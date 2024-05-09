package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class ClimberSS extends SubsystemBase{

    private TalonFX m_ClimberLeadMotor;
    private TalonFX m_ClimberFollowMotor;

    
    public ClimberSS() {
        m_ClimberLeadMotor = new TalonFX(ClimberConstants.CLIMBER_LEAD_ID);
        m_ClimberLeadMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_ClimberLeadMotor.setNeutralMode(NeutralModeValue.Brake);

        m_ClimberFollowMotor = new TalonFX(ClimberConstants.CLIMBER_FOLLOW_ID);
        m_ClimberFollowMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_ClimberFollowMotor.setNeutralMode(NeutralModeValue.Brake);;
        m_ClimberFollowMotor.setInverted(true);
        
    }

     public enum Mode{
        ManualUp,
        ManualDown,
        ManualStop,
        RightUp,
        RightDown,
        RightStop,
        LeftUp,
        LeftDown,
        LeftStop,


    }

    Mode ClimberMode = Mode.ManualStop;
    
    @Override

    public void periodic() {

        switch(ClimberMode) {

            case ManualUp:{
                m_ClimberLeadMotor.set(ClimberConstants.MANUAL_SPEED);
                m_ClimberFollowMotor.set(ClimberConstants.MANUAL_SPEED);
                break;
            }

            case ManualDown:{
                m_ClimberLeadMotor.set(-ClimberConstants.MANUAL_SPEED);
                m_ClimberFollowMotor.set(-ClimberConstants.MANUAL_SPEED);
                break;
            }

            case ManualStop:{
                m_ClimberLeadMotor.set(0);
                m_ClimberFollowMotor.set(0);
                break;
            }


            case RightUp:{
                m_ClimberFollowMotor.set(ClimberConstants.MANUAL_SPEED);
                break;
            }

            case RightDown:{
                m_ClimberFollowMotor.set(-ClimberConstants.MANUAL_SPEED);
                break;
            }

            case RightStop:{
                m_ClimberFollowMotor.set(0.0);
                break;
            }
            

            case LeftUp:{
                m_ClimberLeadMotor.set(ClimberConstants.MANUAL_SPEED);
                break;
            }

            case LeftDown:{
                m_ClimberLeadMotor.set(-ClimberConstants.MANUAL_SPEED);
                System.out.println("mode");
                break;
            }

            case LeftStop:{
                m_ClimberLeadMotor.set(0.0);
                break;
            }


        }

    }

    public void ManualUp(){
        ClimberMode = Mode.ManualUp;
    }
    
    public void ManualDown(){
        ClimberMode = Mode.ManualDown;
    }
    
    public void ManualStop(){
        ClimberMode = Mode.ManualStop;
    }

    //######################################################

    public void RightUp(){
        ClimberMode = Mode.RightUp;
    }
    
    public void RightDown(){
        ClimberMode = Mode.RightDown;
    }
    
    public void RightStop(){
        ClimberMode = Mode.RightStop;
    }

    //#######################################################

    public void LeftUp(){
        ClimberMode = Mode.LeftUp;
    }
    
    public void LeftDown(){
        ClimberMode = Mode.LeftDown;
    }
    
    public void LeftStop(){
        ClimberMode = Mode.LeftStop;
    }
    


     

    
    
}



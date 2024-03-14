package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class ShooterSS extends SubsystemBase {

    private TalonFX m_shooterLeadMotor;
    private TalonFX m_shooterFollowMotor;
    private double speed = 0;

  
    public ShooterSS(){
            m_shooterLeadMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_1_ID);
            m_shooterLeadMotor.getConfigurator().apply(new TalonFXConfiguration());
            m_shooterLeadMotor.setNeutralMode(NeutralModeValue.Brake);
            m_shooterLeadMotor.setInverted(false);

            m_shooterFollowMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_2_ID);
            m_shooterFollowMotor.getConfigurator().apply(new TalonFXConfiguration());
            m_shooterFollowMotor.setNeutralMode(NeutralModeValue.Brake);
            m_shooterFollowMotor.setInverted(false);
            m_shooterFollowMotor.setControl(new StrictFollower(m_shooterLeadMotor.getDeviceID()));
    }


    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode ShooterMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(ShooterMode) {

            case Stop:{
                m_shooterLeadMotor.set(0);
                break;
            }

            case SetSpeed:{
                m_shooterLeadMotor.set(speed);
                break;
            }
        }

        SmartDashboard.putNumber("ShooterSetSpeed M/S", speed);
        SmartDashboard.putNumber("ShooterCurrentSpeed M/S", m_shooterLeadMotor.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(double speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    
    
}
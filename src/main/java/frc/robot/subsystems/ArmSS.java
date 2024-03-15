package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class ArmSS extends SubsystemBase{

    private TalonFX m_ArmLeadMotor;
    private TalonFX m_ArmFollowMotor;
    private CANcoder m_ArmCANcoder;

    private PIDController ArmPIDController;
    private SimpleMotorFeedforward ArmFFController;

    private final double kP = 15;
    private final double kI = 0.5;
    private final double kD = 0;

    private final double kS = 0;
    private final double kV = 0;

    private double output;
    private double setPoint;
    private double maxSpeed;

    private double armVal;


    
    public ArmSS() {
        m_ArmLeadMotor = new TalonFX(ArmConstants.ARM_LEAD_ID);
        m_ArmLeadMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_ArmLeadMotor.setInverted(true);
        m_ArmLeadMotor.setNeutralMode(NeutralModeValue.Brake);

        m_ArmFollowMotor = new TalonFX(ArmConstants.ARM_FOLLOW_ID);
        m_ArmFollowMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_ArmFollowMotor.setNeutralMode(NeutralModeValue.Brake);
        m_ArmFollowMotor.setInverted(false);
        m_ArmFollowMotor.setControl(new StrictFollower(m_ArmLeadMotor.getDeviceID()));

        m_ArmCANcoder = new CANcoder(ArmConstants.ARM_ENCODER_ID);
        m_ArmCANcoder.getConfigurator().apply(Robot.ctreConfigs.armCANcoderConfig);
        m_ArmCANcoder.setPosition(m_ArmCANcoder.getAbsolutePosition().getValueAsDouble());

        ArmPIDController = new PIDController(kP, kI, kD);
        ArmPIDController.setTolerance(0.3);
        ArmPIDController.setIZone(0.5);
        ArmPIDController.setIntegratorRange(0, 0.3);

        ArmFFController = new SimpleMotorFeedforward(kS, kV);
        
    }

     public enum Mode{
        ManualUp,
        ManualDown,
        ManualStop,
        Manual,
        ManualFeedForward,
        PID,
        AutoAim,

    }

    Mode ArmMode = Mode.ManualStop;
    
    @Override

    public void periodic() {

        switch(ArmMode) {

            case ManualUp:{
                m_ArmLeadMotor.set(ArmConstants.MANUAL_SPEED);
                break;
            }

            case ManualDown:{
                m_ArmLeadMotor.set(-ArmConstants.MANUAL_SPEED);
                break;
            }


            case ManualStop:{
                m_ArmLeadMotor.set(0);
                break;
            }

            case Manual:{
                output = MathUtil.clamp(armVal, -Constants.MAX_PID_OUTPUT, Constants.MAX_PID_OUTPUT);
                m_ArmLeadMotor.set(output);
                break;
            }


            case ManualFeedForward:{
                output = MathUtil.clamp((ArmFFController.calculate(1, 0.5)), 
                                                -maxSpeed, maxSpeed);
                m_ArmLeadMotor.set(output);
            }


            case PID:{
                ArmPIDController.reset();
                output = -MathUtil.clamp((ArmPIDController.calculate(m_ArmCANcoder.getPosition().getValueAsDouble(), setPoint) + 
                                                (ArmFFController.calculate(1, 0.5))),
                                                -maxSpeed, maxSpeed);
                m_ArmLeadMotor.set(output);
                break;
            }

            case AutoAim:{
                break;
            }

        }

        SmartDashboard.putNumber("Arm Output", output);
        SmartDashboard.putNumber("Arm setPoint", setPoint);
        SmartDashboard.putNumber("Arm Encoder Pose", m_ArmCANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm AbsEncoder Pose", m_ArmCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public void ManualUp(){
        ArmMode = Mode.ManualUp;
    }
    
    public void ManualDown(){
        ArmMode = Mode.ManualDown;
    }
    
    public void ManualStop(){
        ArmMode = Mode.ManualStop;
    }

    public void Manual(double armVal){
        this.armVal = armVal;
        ArmMode = Mode.Manual;
    }
    
    public void ManualFeedForward(){
        ArmMode = Mode.ManualFeedForward;
    }
    
    public void PID(double setPoint, double maxSpeed){
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        ArmPIDController.reset();
        ArmMode = Mode.PID;
    }

    public double returnSetPoint(){
        return setPoint;
    }

    public Boolean atSetPoint(){
        return ArmPIDController.atSetpoint();
    }
    
    
}



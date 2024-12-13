package frc.robot.subsystems;

import static frc.robot.Constants.MAX_PID_OUTPUT;

import java.sql.DriverManager;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Vision.LimelightHelpers;

//WRIST SUBSYSTEM CLASS THAT HAS THE SAME PROPERTIES OF THE SUBSYSTEM BASE (EXTENDS SUBSYSTEM BASE)

public class WristSS extends SubsystemBase{

/*  CREATING ALL OBJECTS NEEDED FOR THE SUBSYTEM
*   THIS INCLUDES ALL MOTORS, ENCODERS, VARIABLES, AND PID CONTROLLERS
*/

    private static VelocityDutyCycle WristVelocity;

    private final TalonFX m_WristMotor;
    private final CANcoder m_WristCANcoder;

    private final PIDController WristPIDController1;
    private final PIDController WristPIDController2;
    private final SimpleMotorFeedforward WristFFController;

    private final double kP1 = 3; //1.5
    private final double kI1 = 0.35; //0.1
    private final double kD1 = 0.0;//0.00001

    private final double kP2 = 4; //1.15
    private final double kI2 = 0.1; //0.4
    private final double kD2 = 0.0;

    private final double kS = 0.0;
    private final double kV = 0.0;

    private double output;
    private double setPoint;
    private double maxSpeed;

    private double wristVal;

    public double y;

    static boolean disableAutoAim = false;


    /* change to true to use linear interpolation when auto aiming */
    boolean linearInterpolation = false;


    // CONFIGURING ALL MOTORS, ENCODERS, AND PID CONTOLLERS
    public WristSS() {
        m_WristMotor = new TalonFX(WristConstants.WRIST_MOTOR_ID);
        m_WristMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_WristMotor.setNeutralMode(NeutralModeValue.Brake);
        

        m_WristCANcoder = new CANcoder(WristConstants.WRIST_ENCODER_ID);
        m_WristCANcoder.getConfigurator().apply(Robot.ctreConfigs.wristCANcoderConfig);
        m_WristCANcoder.setPosition(m_WristCANcoder.getAbsolutePosition().getValueAsDouble());

        /* SETTING kP, ki, and kD VALUES FOR PID CONTROLLER
         * NORMALLY THERE SHOULD ONLY BE ONE PID CONTROLLER
        */
        WristPIDController1 = new PIDController(kP1, kI1, kD1);
        WristPIDController1.setTolerance(0.005);
        WristPIDController1.setIZone(0.5);
        WristPIDController1.setIntegratorRange(0, 0.4);

        WristPIDController2 = new PIDController(kP2, kI2, kD2);
        WristPIDController2.setTolerance(0.05);
        WristPIDController2.setIZone(0.5);
        WristPIDController2.setIntegratorRange(0, 0.3);

        WristFFController = new SimpleMotorFeedforward(kS, kV);     
        
        WristVelocity = new VelocityDutyCycle(0);
    }

    // CREATING A LIST OF ACCEPTABLE MODES
     public enum Mode{
        ManualUp,
        ManualDown,
        ManualStop,
        Manual,
        ManualFeedForward,
        PID,
        AutoAim,

    }

    // MAKING A WRISTMODE OBJECT EQUAL TO THE MODE.  ALSO SETS STARTING MODE TO MANUALSTOP
    Mode WristMode = Mode.ManualStop;
    

    // PERIODIC METHOD THAT WILL RUN CONTINUOUSLY 
    @Override
    public void periodic() {

        /*  I GRAPHED VALUES OF TY AND THEIR CORRESPONDING WRIST SETPOINTS
         *  THIS GAVE ME A LINEAR EQUATION THAT I CAN INTERPOLATE A NEW WRIST SETPOINT BASED ON THE TY VALUE
         *  STORES THE INTERPOLATED VALUE IN THE Y VARIABLE IF A VALID TAG IS SEEN
         *  IF THE TAG IS NOT 7 OR 4 Y IS SET TO THE SPEAKER_POS
        */
        if(!disableAutoAim){
            if(LimelightHelpers.getFiducialID("limelight") == 7 || LimelightHelpers.getFiducialID("limelight") == 4){
                if(linearInterpolation){
                    double x = LimelightHelpers.getTY("limelight");
                    y = LimelightConstants.lA * x + LimelightConstants.lB;
                }
                else{
                    double x = LimelightHelpers.getTY("limelight");
                    y = (LimelightConstants.qA * Math.pow(x, 2)) + (LimelightConstants.qB * x) + (LimelightConstants.qC);
                }

                if(LimelightHelpers.getTY("limelight") < -20){
                    if(DriverStation.isAutonomousEnabled()){
                        y += 0.003;
                    }
                    else{
                        // y += 0.008;
                        y += 0.003;
                    }
                }
                    y += 0.004;

            }
            else{
                y = WristConstants.SPEAKER_POS;
            }   
        }
        else{
            y = WristConstants.SPEAKER_POS;
        }
        // SWITCHES THE MODE.  ONLY THE SELECTED MODE WILL RUN PERIODICLY.
        switch(WristMode) {

            // SETS THE WRIST MOTOR SPEED TO WRISTCONSTANTS.MANUAL_SPEED 
            case ManualUp:{
                m_WristMotor.set(WristConstants.MANUAL_SPEED);
                break;
            }

            case ManualDown:{
                m_WristMotor.set(-WristConstants.MANUAL_SPEED);
                break;
            }

            case ManualStop:{
                m_WristMotor.setControl(WristVelocity.withVelocity(0));
                break;
            }

            /* TAKES THE VALUE OF A JOYSTICK STORED IN wristVal AND SETS THE MOTOR OUTPUT EQUAL TO IT
             * CLAMPS THE VALUE TO BE BETWEEN +- Constants.MAX_PID_OUTPUT
             */
            case Manual:{
                output = MathUtil.clamp(wristVal, -Constants.MAX_PID_OUTPUT, Constants.MAX_PID_OUTPUT);
                m_WristMotor.set(output);
                break;
            }

            case ManualFeedForward:{
                output = MathUtil.clamp((WristFFController.calculate(0.5, 0.5)), 
                                                -maxSpeed, maxSpeed);
                m_WristMotor.set(output);
            }

            /* TAKES THE CURRENT ENCODER POSTION AND THE DESIRED SETPOINT AND SETS THE MOTOR OUTPUT TO SMOOTHLY AND QUICKLY REACH THE SETPOINT
             * CLAMPS THE PID CONTROLLER BETWEEN -+ MAX_PID_OUTPUT. 
            */
            case PID:{
                WristPIDController1.reset();
                output = MathUtil.clamp((WristPIDController1.calculate(m_WristCANcoder.getAbsolutePosition().getValueAsDouble(), setPoint) + 
                                                (WristFFController.calculate(1, 0.5))),
                                                -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
                m_WristMotor.set(output);
                break;
            }

            /* SETS THE SETPOINT EQUAL TO THE INTERPOLATED WRIST VALUE
             * CALCULATES THE PID AND SETS THE MOTOR TO THE OUTPUT
             */
            case AutoAim:{
                setPoint = y;
                output = MathUtil.clamp((WristPIDController1.calculate(m_WristCANcoder.getAbsolutePosition().getValueAsDouble(), setPoint) + 
                                                (WristFFController.calculate(1, 0.5))),
                                                -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
                m_WristMotor.set(output);

            }

        }

        /*  DISPLAYS IMPORTANT INFORMATION TO THE SMARTDASHBOARD */
        SmartDashboard.putNumber("Wrist Output", output);
        SmartDashboard.putNumber("Wrist setPoint", setPoint);
        SmartDashboard.putNumber("Wrist Pose", m_WristCANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist AbsPose", m_WristCANcoder.getAbsolutePosition().getValueAsDouble());

        SmartDashboard.putNumber("Wrist Angle", y);
        SmartDashboard.putNumber("FiducialID", LimelightHelpers.getFiducialID("limelight"));
        SmartDashboard.putBoolean("Auto", DriverStation.isAutonomousEnabled());
        SmartDashboard.putBoolean("toggleAutoAim", !disableAutoAim);

    }

    /* METHODS THAT CAN BE CALLED TO RUN */

    /* CHANGES THE MODE TO MANUAL UP */
    public void ManualUp(){
        WristMode = Mode.ManualUp;
    }
    /* CHANGES THE MODE TO MANUAL DOWN */
    public void ManualDown(){
        WristMode = Mode.ManualDown;
    }
    /* CHANGES THE MODE TO MANUAL STOP */
    public void ManualStop(){
        WristMode = Mode.ManualStop;
    }
    /* CHANGES THE MODE TO MANUAL */
    public void Manual(double WristVal){
        this.wristVal = WristVal;
        WristMode = Mode.Manual;
    }
    
    public void ManualFeedForward(){
        WristMode = Mode.ManualFeedForward;
    }
    
    /*  METHOD THAT REQUIRES
     *  SETS setPoint IN METHOD TO EQUAL setPoint IN CLASS
     *  SETS maxSpeed IN METHOD TO EQUAL maxSpeed IN CLASS
     *  RESETS PID CONTROLLER
     *  SETS WRISTMODE TO PID
     * 
     *  @param setPoint DISIRED ENCODER LOCATION
     *  @param maxSpeed MAX OUTPUT FOR PID CONTROL
     *  @return NOTHING
    */
    public void PID(double setPoint, double maxSpeed){
        this.maxSpeed = maxSpeed;
        this.setPoint = setPoint;
        WristPIDController1.reset();
        WristMode = Mode.PID;
    }

    /* METHOD THAT HAS NO PARAMETERS
     * @returns setPoint/DESIRED ENCODER LOCATION
     */
    public double returnSetPoint(){
        return setPoint;
    }

    public Boolean atSetPoint(){
        return WristPIDController1.atSetpoint();
    }

    public void AutoAim(){
        WristPIDController1.reset();
        WristMode = Mode.AutoAim;
    }

    public void toggleAutoAimState(){
        if(disableAutoAim == false){
            disableAutoAim = true;
        }
        else{
            disableAutoAim = false;
        }
    }

    public static boolean getAutoAim(){
        return disableAutoAim;
    }

 



    
    
}



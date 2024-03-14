package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/* SEE WristSS FOR EXPLANATIONS */

public class LEDSS extends SubsystemBase {

    public LEDSS(){
        HttpCamera httpCamera = new HttpCamera("CoprocessorCamera", "http://10.32.76.11:5801");
        CameraServer.addCamera(httpCamera);
        Shuffleboard.getTab("Tab")
        .add(httpCamera);
    }


    public enum Mode{
        Off,
        On,
        Blink
    }

    Mode LL_LEDMode = Mode.Off;
    
    @Override

    public void periodic() {

        switch(LL_LEDMode) {

            case Off:{
                LimelightHelpers.setLEDMode_ForceOff("");
                break;
            }

            case On:{
                LimelightHelpers.setLEDMode_ForceOn("");
                break;
            }

            case Blink:{
                LimelightHelpers.setLEDMode_ForceBlink("");
                break;
            }
        }

    }

    public void Off(){
        LL_LEDMode = Mode.Off;
    }

    public void On(){
        LL_LEDMode = Mode.On;
    }
    
    public void Blink(){
        LL_LEDMode = Mode.Blink;
    }
    
    
}
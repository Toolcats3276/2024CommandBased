package frc.robot.commands.z_WristCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.WristSS;

public class WristManualCmd extends Command {
    
    private WristSS s_Wrist;
    private DoubleSupplier WristSup;




    public WristManualCmd(WristSS s_Wrist, DoubleSupplier WristAxis) {
        this.s_Wrist = s_Wrist;
        this.WristSup = WristAxis;
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double WristVal = MathUtil.applyDeadband(WristSup.getAsDouble(), Constants.stickDeadband)/4;

        if(WristVal == 0){
            s_Wrist.ManualStop();
        }
        else{
            s_Wrist.Manual(WristVal);
        }
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

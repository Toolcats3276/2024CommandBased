package frc.robot.commands.z_ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSS;

public class ArmManualCommand extends Command {
    
    private ArmSS s_Arm;
    private DoubleSupplier ArmSup;



    public ArmManualCommand(ArmSS s_Arm, DoubleSupplier ArmAxis) {
        this.s_Arm = s_Arm;
        this.ArmSup = ArmAxis;
        addRequirements(s_Arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double ArmVal = MathUtil.applyDeadband(ArmSup.getAsDouble(), Constants.stickDeadband)/2;

        if(ArmVal == 0){
            s_Arm.ManualStop();
        }
        else{
            s_Arm.Manual(ArmVal);
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

package frc.robot.commands.TeleopCommands.BaseCommands;

import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoAimSwerve extends Command {    
    private Swerve s_Swerve;    
    Debouncer autoAimDebouncer;

    public AutoAimSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        autoAimDebouncer = new Debouncer(0.15);
        addRequirements(s_Swerve);

    }

    @Override
    public void execute() {
        double rotationVal;
        double translationVal;
        double strafeVal;
        boolean robotCentricVal;
        

        translationVal = 0;
        strafeVal = 0;
        rotationVal = MathUtil.applyDeadband(Swerve.LLAngularVelocity(), Constants.stickDeadband);
        robotCentricVal = false;
        

        SmartDashboard.putNumber("rotation", rotationVal);
        SmartDashboard.putBoolean("Auto Aim", Swerve.getAutoAimState());


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal,
            !robotCentricVal,
            true
        );
    }

    @Override
    public boolean isFinished(){
        return autoAimDebouncer.calculate(Math.abs(LimelightHelpers.getTX("limelight")) < 2.5);

    }
}
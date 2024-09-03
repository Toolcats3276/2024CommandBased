package frc.robot.commands.TeleopCommands.BaseCommands;

import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    // private BooleanSupplier LLAimSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        // this.LLAimSup = LLAimSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        // boolean robotCentricVal = robotCentricSup.getAsBoolean();
        
        SmartDashboard.putNumber("rotation", rotationVal);
        // SmartDashboard.putBoolean("Auto Aim", LLAimSup.getAsBoolean());


        


        // // while the LLAimSup is true, overwrite some of the driving values with the output of our limelight methods
        // if(LLAimSup.getAsBoolean()){
        //     rotationVal = Swerve.LLAngularVelocity();
        //     translationVal = Swerve.LLRangeVelocity();
        //     //while using Limelight, turn off field-relative driving.
        //     robotCentricVal = false;
        //     System.out.println("auto aim");
        // }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(),
            true
        );
    }
}
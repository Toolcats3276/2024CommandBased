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

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        double rotationVal;
        double translationVal;
        double strafeVal;
        boolean robotCentricVal;
        
        // /* AIMING AND RANGING WITH SWERVE
        // *  while the LLAimSup is true, overwrite some of the driving values with the output of our limelight methods
        // */
        // if(LLAimSup.getAsBoolean()){
        //     rotationVal = Swerve.LLAngularVelocity();
        //     translationVal = Swerve.LLRangeVelocity();
        //     strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        //     //while ranging with Limelight, turn on robot-centric driving.
        //     robotCentricVal = true;
        //     System.out.println("auto aim");
        // }

        /* AIMING WITH SWERVE
        *  while the LLAimSup is true, overwrite some of the driving values with the output of our limelight methods
        */
        if(Swerve.getAutoAimState()){
        /* Get Values, Deadband */
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) + Swerve.LLAngularVelocity();
            robotCentricVal = robotCentricSup.getAsBoolean();
        }
        else{
        /* NORMAL TELEOP DRIVING COMMAND */
        /* Get Values, Deadband */
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            robotCentricVal = robotCentricSup.getAsBoolean();
        }
        

        SmartDashboard.putNumber("rotation", rotationVal);
        SmartDashboard.putBoolean("Auto Aim", Swerve.getAutoAimState());


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricVal,
            true
        );
    }
}
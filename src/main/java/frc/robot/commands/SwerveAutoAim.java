package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class SwerveAutoAim extends Command {
    
    private Swerve s_Swerve;
    private double Rotation;


    public SwerveAutoAim(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        Rotation = -(LimelightHelpers.getTX("")) * (Math.PI/180);

    }

    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(0.0, 0.0), Rotation, false, false);;
        
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX("")) < 0.5;

    }
    
}

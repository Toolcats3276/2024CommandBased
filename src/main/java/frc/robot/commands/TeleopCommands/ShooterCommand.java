package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSS;

public class ShooterCommand extends Command {
    
    private ShooterSS s_Shooter;
    private Double speed;


    public ShooterCommand(ShooterSS s_Shooter, Double speed) {
        this.s_Shooter = s_Shooter;
        this.speed = speed;
        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        s_Shooter.setSpeed(speed);
    }

    @Override
    public void execute() {


    }

    @Override
    public void end(boolean interrupted) {

        }


    @Override
    public boolean isFinished() {
            return true;
        }
    }
    


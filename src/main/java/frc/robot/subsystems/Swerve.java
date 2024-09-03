package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* SEE WristSS FOR EXPLANATIONS */

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Rotation2d storedHeading = new Rotation2d();
    private boolean AutoRotationState;

    private final PIDController AutoRotationPID;

    public 

    StructPublisher<Pose2d> publisher;
    StructArrayPublisher<Pose2d> arrayPublisher;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        AutoRotationPID = new PIDController(
            0.05, 0, 0);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
       
        // Configure AutoBuilder last
        /* CREATES AUTOBUILDER OBJECT WITH A HOLONOMIC DRIVETRAIN (SWERVE)
         * @param robotPose
         * @param setPose METHOD THAT RESTS THE ROBOT POSITON
         * @param getSpeeds GETS ROBOT RELATIVE SPEEDS
         * @param driveRobotRelative METHOD TO DRIVE ROBOT REALTIVE
         * @param pathFollowerConfig WHERE THE PID FOR TRANSLATION AND ROTATION IS SET
         * @param mirrorPath
         */
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::setPose, 
            this::getSpeeds, 
            this::driveRobotRelative,
            Constants.AutoConstants.pathFollowerConfig, 
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);

        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
        arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    MathUtil.clamp(rotation + this.shuttleHeading(), -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity), 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    MathUtil.clamp(rotation + this.shuttleHeading(), -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity))
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose(){
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getLLPose(){
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setLLPose(Pose2d pose){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }
 
    public void storeHeading(){
        storedHeading = getPose().getRotation();
    }

    public Rotation2d getStoredHeading(){
        return storedHeading;
    }

    public void setTrapHeading(){
        if(115.0 < storedHeading.getDegrees() || storedHeading.getDegrees() < 125.0){
            setHeading(new Rotation2d(240));
        }
        // else if(-5.0 < storedHeading.getDegrees() || storedHeading.getDegrees() < 5.0){
        //     setHeading(new Rotation2d(120));
        // }
        else if(-115.0 < storedHeading.getDegrees() || storedHeading.getDegrees() < -125.0){
            setHeading(new Rotation2d(0));
        }
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public double alternateShuttleSetPoint(){
        var alliance = DriverStation.getAlliance();
            if (alliance.get() == DriverStation.Alliance.Red){
                return 27;
            }
            else{
                return -27;
            }
    }

    public double shuttleHeading(){
        if(this.getAutoRotationState()){
            return AutoRotationPID.calculate(this.getHeading().getDegrees(), alternateShuttleSetPoint());
        }
        else{
            return 0;
        }
    }

    public void setAutoRotationState(boolean AutoRotationState){
        this.AutoRotationState = AutoRotationState;
    }

    public boolean getAutoRotationState(){
        return AutoRotationState;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
      }
      
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }


    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    public static double LLAngularVelocity(){
        if(LimelightHelpers.getTargetCount("limelight") == 1){
            // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
            // your limelight 3 feed, tx should return roughly 31 degrees.
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * LimelightConstants.AIM_KP;

            // convert to radians per second for our drive method
            targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

            //invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;

            return targetingAngularVelocity;
        }
        else return 0;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public static double LLRangeVelocity(){   
        if(LimelightHelpers.getTargetCount("limelight") == 1){ 
            double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * LimelightConstants.RANGE_KP;
            targetingForwardSpeed *= Constants.Swerve.maxSpeed;
            targetingForwardSpeed *= -1.0;
            return targetingForwardSpeed;
        }
        else return 0;
    }


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

            SmartDashboard.putNumber("Heading", getHeading().getDegrees());
            SmartDashboard.putNumber("Yaw", getGyroYaw().getDegrees());
            SmartDashboard.putNumber("StoredHeading", storedHeading.getDegrees());
            SmartDashboard.putBoolean("Rotate State", AutoRotationState);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);   
             
        }

            SmartDashboard.putNumber("Targets", LimelightHelpers.getTargetCount("limelight"));
    }



}
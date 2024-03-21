package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
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

    public double tl = LimelightHelpers.getLatency_Pipeline("");
    public double cl = LimelightHelpers.getLatency_Capture("");
    public double latency;

    public Pose2d PoseA;
    public Pose2d PoseB;

    public Rotation2d storedHeading = new Rotation2d();

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

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        /*CREATES A swervePoseEstimator THAT INTEGRATES ODOMETRY WITH VISION MEASURMENTS
         * 
         * @param swerveKinematics
         * @param gyroYaw
         * @param modualePositions
         * @param robot pose
         * @param Standard Deviation OF SWERVE ODOMETRY
         * @param Standard Deviation OF VISION MEASURMENTS
         */
        swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            getPose(),
            createStateStdDevs(
                Constants.kPositionStdDevX,
                Constants.kPositionStdDevY,
                Constants.kPositionStdDevTheta),
            createVisionMeasurementStdDevs(
                Constants.kVisionStdDevX,
                Constants.kVisionStdDevY,
                Constants.kVisionStdDevTheta));
       
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
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
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

    public void setTrapHeading(){
    //     if(115.0 < storedHeading.getDegrees() || storedHeading.getDegrees() < 125.0){
    //         setHeading(new Rotation2d(240));
    //     }
    //     // else if(-5.0 < storedHeading.getDegrees() || storedHeading.getDegrees() < 5.0){
    //     //     setHeading(new Rotation2d(120));
    //     // }
    //     else if(-115.0 < storedHeading.getDegrees() || storedHeading.getDegrees() < -125.0){
    //         setHeading(new Rotation2d(0));
    //     }
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
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


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        // latency = Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0);
        // if(LimelightHelpers.getTV("")){
        // swervePoseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(""), latency);
        // }
        // swervePoseEstimator.update(getGyroYaw(), getModulePositions());

        // publisher.set(PoseA);
        // arrayPublisher.set(new Pose2d[] {PoseA, PoseB});
            SmartDashboard.putNumber("Heading", getHeading().getDegrees());
            SmartDashboard.putNumber("Yaw", getGyroYaw().getDegrees());
            SmartDashboard.putNumber("StoredHeading", storedHeading.getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);   
             
        }
    }


      /**
   * Creates a vector of standard deviations for the states. Standard deviations of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }


  /**
   * Creates a vector of standard deviations for the local measurements. Standard deviations of
   * encoder and gyro rate measurements. Increase these numbers to trust sensor readings from
   * encoders and gyros less.
   *
   * @param theta in degrees per second
   * @param s std for all module positions in meters per sec
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
    return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
  }


  /**
   * Creates a vector of standard deviations for the vision measurements. Standard deviations of
   * global measurements from vision. Increase these numbers to trust global measurements from
   * vision less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }
}
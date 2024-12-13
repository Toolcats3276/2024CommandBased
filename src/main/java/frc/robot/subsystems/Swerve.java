package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightUpdater;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.sql.Driver;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* SEE WristSS FOR EXPLANATIONS */

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public static SwerveDrivePoseEstimator m_poseEstimator;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;

    public Rotation2d storedHeading = new Rotation2d();
    private boolean AutoRotationState;
    private static boolean AutoAimState;

    private static PIDController AutoRotationPID;
    private static PIDController LLRotationPID;
    private static PIDController LLTranslationPID;

    public LimelightUpdater leftLimelight;
    public LimelightUpdater rightLimelight;

    Matrix<N3,N1> megaTagStdDev;
    boolean useMegaTag1 = true;

    private Field2d m_EstimatedPose;
    private Field2d m_OdometryPose;

    public static Debouncer autoAimDebouncer;
    
    public Swerve() {

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        autoAimDebouncer = new Debouncer(0.1);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        AutoRotationPID = new PIDController(
            0.06, 0.0, 0.0); //0.06, 0, 0
        LLRotationPID = new PIDController(
            0.0007, 0, 0);
        LLTranslationPID = new PIDController(
            0.004, 0, 0);
                
            
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
            
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(99999, 99999, Units.degreesToRadians(5)),
            VecBuilder.fill(0.000005, 0.000005, Units.degreesToRadians(30)));

        leftLimelight = new LimelightUpdater("limelight-left", LimelightConstants.MEGA_TAG_1_DISABLED_STD_DEV, true);
        rightLimelight = new LimelightUpdater("limelight-right", LimelightConstants.MEGA_TAG_1_DISABLED_STD_DEV, true);

        m_EstimatedPose = new Field2d();
        m_OdometryPose = new Field2d();

        // if(!(mt1 == null)){
        //     useMegaTag2 = false;
        //     updatePose();
        //     setPose(m_poseEstimator.getEstimatedPosition());
        //     useMegaTag2 = true;
        // }

        /*sets valid tag ids for localization */
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", LimelightConstants.VALID_TAG_ID);


        /* Sets the wanted method for overriding Pathplanner's rotation target */
        // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        // Configure AutoBuilder last
        /* CREATES AUTOBUILDER OBJECT WITH A HOLONOMIC DRIVETRAIN (SWERVE)
         * robotPose
         * setPose METHOD THAT RESETS THE ROBOT POSITON
         * getSpeeds GETS ROBOT RELATIVE SPEEDS
         * driveRobotRelative METHOD TO DRIVE ROBOT REALTIVE
         * pathFollowerConfig WHERE THE PID FOR TRANSLATION AND ROTATION IS SET
         * mirrorPath
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

        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    MathUtil.clamp(rotation + this.shuttleHeading() + LLAngularVelocity(), -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity), 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    MathUtil.clamp(rotation + this.shuttleHeading() + LLAngularVelocity(), -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity))
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
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
        // return swerveOdometry.getPoseMeters();
        return m_poseEstimator.getEstimatedPosition();
    }


    public void setPose(Pose2d pose){
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }


    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
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

    public static boolean isSpeakerVisable(){
        if(LimelightHelpers.getFiducialID("limelight-left") == 7 || LimelightHelpers.getFiducialID("limelight-left") == 4){
            return true;
        }
        else{
            return false;
        }
    }


    // simple PID turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    public static double LLAngularVelocity(){
        double targetingAngularVelocity;
        if(!WristSS.getAutoAim()){
            if(getAutoAim()){
                // targetingAngularVelocity = LLRotationPID.calculate(LimelightHelpers.getTX("limelight"), 0);
                targetingAngularVelocity = AutoRotationPID.calculate(LimelightHelpers.getTX("limelight-left"), 0);
                // targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;
            }
            // else if (getAutoAimState()){
            //     targetingAngularVelocity = AutoRotationPID.calculate(, 0);
            // }
            else{
                targetingAngularVelocity = 0;
            }
        }
        else{
            targetingAngularVelocity = 0;
        }
        return targetingAngularVelocity;
    }


    // simple PID ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public static double LLRangeVelocity(){  
        double targetingForwardVelocity;
        if(LimelightHelpers.getTV("limelight-left") & getAutoAimState()){
            targetingForwardVelocity = LLTranslationPID.calculate(LimelightHelpers.getTY("limelight-left"), LimelightConstants.SPEAKER_TY);
            targetingForwardVelocity *= Constants.Swerve.maxSpeed;
        }
        else{
            targetingForwardVelocity = 0;
        }
        return -targetingForwardVelocity;
    }

    public void setAutoAimState(boolean AutoAimState){
        Swerve.AutoAimState = AutoAimState;
    }

    public static boolean getAutoAimState(){
        return AutoAimState;
    }

    public static boolean getAutoAim(){
        return autoAimDebouncer.calculate(AutoAimState & isSpeakerVisable());
    }

    /* Currently unused rotation override for pathplanner
     * Creates a Rotation2d with the apriltage tX value 
     * This SHOULD orient the robot toward the speaker
     */
    // public Optional<Rotation2d> getRotationTargetOverride(){
    //     // Some condition that should decide if we want to override rotation
    //     if(isSpeakerVisable() & getAutoAimState()){
    //         // Return an optional containing the rotation override (this should be a field relative rotation)
    //         return Optional.of(Rotation2d.fromDegrees(getHeading().getDegrees() - LimelightHelpers.getTX("limelight")));
    //     } else {
    //         // return an empty optional when we don't want to override the path's rotation
    //         return Optional.empty();
    //     }
    // }


    /* requires a matrix for the standard deviation of mega tag 1 and mega tag 2 vision measurements
     * this lets us change StdDev depending on robot state
     * examples: we want the StdDev to be really low during disabled to adjust for imprecise placement of the robot
     *           we might want StdDev to scale propotionaly with distance
     */
    // public void setStdDev(Matrix<N3,N1> megaTagStdDev){
    //     this.megaTagStdDev = megaTagStdDev;
    // }

    // public Matrix<N3,N1> scaleStdDevByError(){
        
    //     Transform2d errorTransform = m_poseEstimator.getEstimatedPosition().minus(swerveOdometry.getPoseMeters());
    //     Pose2d errorPose = new Pose2d(errorTransform.getTranslation(), errorTransform.getRotation());
    //     Matrix<N3,N1> scaledStdDev = VecBuilder.fill(errorPose.getX() * 0.1, errorPose.getY() * 0.1, errorPose.getRotation().getDegrees() * 0.1);
    //     return scaledStdDev;
    // }

    /* useMegaTag1 allows us to switch between mega tag 1 and mega tag 2 depending on robot state 
    */
    // public void setMegaTagMode(boolean useMegaTag1){
    //     this.useMegaTag1 = useMegaTag1;
    // }


    // public boolean rejectUpdate(){
    //     boolean doRejectUpdate = false;

    //     /* sets a variable to the robot's pose estimate on the blue side for mega tag 1 and mega tag 2
    //      * you should keep a constant origin at the bottom right of the blue alliance wall
    //      * this keeps +X as forward and +Y as left
    //      */
    //     mt1R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    //     mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    //     mt1L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    //     mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
         
    //     /* checks if the value of mt1 is null so code doesn't crash before the limelight can boot up */
    //     if(useMegaTag1 == true && mt1R != null || useMegaTag1 == true && mt1L != null)
    //     {
    //         if(mt1R.tagCount == 1 && mt1R.rawFiducials.length == 1)
    //         {
    //         if(mt1R.rawFiducials[0].ambiguity > .7)
    //         {
    //             doRejectUpdate = true;
    //         }

    //         if(mt1R.rawFiducials[0].distToCamera > 3)
    //         {
    //             doRejectUpdate = true;
    //         }
    //         }

    //         if(mt1R.tagCount == 0)
    //         {
    //             doRejectUpdate = true;
    //         }
    //     }

    //     else if(useMegaTag1 == false && mt1R != null || useMegaTag1 == false && mt1L != null)
    //     {
    //         LimelightHelpers.SetRobotOrientation("limelight-left", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //         LimelightHelpers.SetRobotOrientation("left", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //         if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //         {
    //           doRejectUpdate = true;
    //         }

    //         if(mt2R.tagCount == 0)
    //         {
    //           doRejectUpdate = true;
    //         }
    //       }

    //     return doRejectUpdate;
    // }


    // /* updates pose estimate
    //  * this should be called periodically
    //  * checks the value of useMegaTag1 to determine which megatag method to use
    //  * rejects any updates to vision data if certain criteria are met
    //  * adds vision measurments to megaTagStdDev set in setStdDev
    //  */

    // // public void updatePose(Matrix<N3,N1> mt1StdDev, Matrix<N3,N1> mt2StdDev, boolean useMegaTag2){
    // public void updatePose(){
        
    //     /* sets a variable to the robot's pose estimate on the blue side for mega tag 1 and mega tag 2
    //      * you should keep a constant origin at the bottom right of the blue alliance wall
    //      * this keeps +X as forward and +Y as left
    //      */
    //     mt1R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    //     mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    //     mt1L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    //     mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

      
    //     //   if(Math.abs(m_poseEstimator.getEstimatedPosition().getX() - swerveOdometry.getPoseMeters().getX()) > 5 || Math.abs(m_poseEstimator.getEstimatedPosition().getY() - swerveOdometry.getPoseMeters().getY()) > 5){


    //     if(!rejectUpdate() && useMegaTag1 == true && mt1R != null && mt1L != null)
    //     {
    //         // /* if estimated pose and odometry get too far off it will reset both to the limelights pose
    //         //  * this should'nt be done like this
    //         // */
    //         // if(m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(swerveOdometry.getPoseMeters().getTranslation()) > 5 
    //         //     || Math.abs(m_poseEstimator.getEstimatedPosition().getRotation().minus(swerveOdometry.getPoseMeters().getRotation()).getDegrees()) > 10){
    //         //         swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), LimelightHelpers.getBotPose2d_wpiBlue(""));
    //         //         m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), LimelightHelpers.getBotPose2d_wpiBlue(""));
    //         // }


    //         // megaTagStdDev = 

    //         m_poseEstimator.setVisionMeasurementStdDevs(megaTagStdDev);
    //         m_poseEstimator.addVisionMeasurement(
    //             mt1R.pose, 
    //             mt1R.timestampSeconds);
    //         m_poseEstimator.addVisionMeasurement(
    //             mt1L.pose,
    //             mt1L.timestampSeconds);
    //     }


    //     if(!rejectUpdate() && useMegaTag1 == false && mt1R != null && mt1L != null)
    //     {
    //         m_poseEstimator.setVisionMeasurementStdDevs(megaTagStdDev);
    //         // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.00000001));
    //         m_poseEstimator.addVisionMeasurement(
    //             mt2R.pose,
    //             mt2R.timestampSeconds);
    //         m_poseEstimator.addVisionMeasurement(
    //             mt2L.pose,
    //             mt2L.timestampSeconds);
    //     }
        

    //     /* the actual method to update the pose estimate
    //      * this will act just like a normal odometry object if no vision measurments are added
    //      */
    //     m_poseEstimator.update(
    //         getGyroYaw(),
    //         getModulePositions()
    //     );
    // }


    public void updatePoseEstimate(){
        m_poseEstimator.update(
            getGyroYaw(), 
            getModulePositions());

            leftLimelight.updatePoseEstimates();
            rightLimelight.updatePoseEstimates();

            if(!leftLimelight.rejectUpdate()){
                m_poseEstimator.setVisionMeasurementStdDevs(leftLimelight.getVisionMeasurementStdDevs());
                m_poseEstimator.addVisionMeasurement(leftLimelight.getPoseEstimate(), leftLimelight.getTimestamp());
            }
            if(!rightLimelight.rejectUpdate()){
                m_poseEstimator.setVisionMeasurementStdDevs(rightLimelight.getVisionMeasurementStdDevs());
                m_poseEstimator.addVisionMeasurement(rightLimelight.getPoseEstimate(), rightLimelight.getTimestamp());
            }
    }

    @Override
    public void periodic(){
        updatePoseEstimate();
        swerveOdometry.update(getGyroYaw(), getModulePositions());


            SmartDashboard.putNumber("Heading", getHeading().getDegrees());
            SmartDashboard.putNumber("Yaw", getGyroYaw().getDegrees());
            SmartDashboard.putBoolean("Rotate State", AutoRotationState);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

            SmartDashboard.putNumber("tX", LimelightHelpers.getTX("limelight-left"));
            SmartDashboard.putNumber("tY", LimelightHelpers.getTY("limelight-left"));
            SmartDashboard.putNumber("targetingAngularVelocity", LLAngularVelocity());
            SmartDashboard.putBoolean("Speaker Visable", isSpeakerVisable());
            SmartDashboard.putBoolean("Auto Aim", getAutoAimState());


            // m_EstimatedPose.setRobotPose(m_poseEstimator.getEstimatedPosition());
            m_EstimatedPose.setRobotPose(m_poseEstimator.getEstimatedPosition());
            SmartDashboard.putData("EstimatedPose", m_EstimatedPose);

            // m_OdometryPose.setRobotPose(swerveOdometry.getPoseMeters());
            m_OdometryPose.setRobotPose(swerveOdometry.getPoseMeters());
            SmartDashboard.putData("OdometryPose", m_OdometryPose);

            SmartDashboard.putNumber("leftTagDistance", leftLimelight.getTagDistance());
            SmartDashboard.putNumber("rightTagDistance", rightLimelight.getTagDistance());



    }
}
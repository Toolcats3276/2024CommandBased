package frc.robot.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Swerve;

public class LimelightUpdater {

    private String limelightName;
    private Matrix<N3,N1> visionMeasurementStdDevs;
    private boolean useMegaTag1;

    private boolean doRejectUpdate;

    LimelightHelpers.PoseEstimate megaTag1PoseEstimator;
    private Pose2d megaTag1Pose2d;
    private double megaTag1Timestamp;

    LimelightHelpers.PoseEstimate megaTag2PoseEstimator;
    private Pose2d megaTag2Pose2d;
    private double megaTag2Timestamp;


    
    /**
   * Constructs a LimelightUpdater to quickly add multiple limelights.
   *
   * @param limelightName The name of the limelight as set in the WebUI.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less (can be changed later).
   * @param useMegaTag1 This will give mega tag 1 estimates if true and mega tag 2 estimates if false (can be changed later).
   */

    public LimelightUpdater(String limelightName, Matrix<N3,N1> visionMeasurementStdDevs, boolean useMegaTag1){
        
        this.limelightName = limelightName;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
        this.useMegaTag1 = useMegaTag1;

        doRejectUpdate = false;


        megaTag1PoseEstimator = new PoseEstimate();
        megaTag1PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        megaTag1Pose2d = new Pose2d();

        megaTag2PoseEstimator = new PoseEstimate();
        megaTag2PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        megaTag2Pose2d = new Pose2d();

    }

    /**
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     * in meters, y position in meters, and heading in radians). Increase these numbers to trust
     * the vision pose measurement less (can be changed later).
     */
    public void setVisionMeasurementStdDevs(Matrix<N3,N1> visionMeasurementStdDevs){
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    /** 
     * @return The standard deviations of the vision pose measurments (x position
     * in meters, y position in meters, and heading in radians). Increase these numbers to trust
     * the vision pose measurement less (can be changed later).
    */
    public Matrix<N3,N1> getVisionMeasurementStdDevs(){
        return visionMeasurementStdDevs;
    }

    /**
     * @param useMegaTag1 This will change weather or not to give mega tag 1 estimates if true and mega tag 2 estimates if false.
     */
    public void setMegaTagMode(boolean useMegaTag1){
        this.useMegaTag1 = useMegaTag1;
    }

    /**
     * @return This will give mega tag 1 estimates if true and mega tag 2 estimates if false.
     */
    public boolean getMegaTagMode(){
        return useMegaTag1;
    }

    /**
     * @return weather or not limelight pose should be updated
     */
    public boolean rejectUpdate(){

        if(useMegaTag1 == true)
        {
            if(megaTag1PoseEstimator != null){

            LimelightHelpers.PoseEstimate megaTag1PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

              if(megaTag1PoseEstimator.tagCount == 1 & megaTag1PoseEstimator.rawFiducials.length == 1)
              {
                if(megaTag1PoseEstimator.rawFiducials[0].ambiguity > .7)
                {
                  doRejectUpdate = true;
                }
                if(megaTag1PoseEstimator.rawFiducials[0].distToCamera > 3)
                {
                  doRejectUpdate = true;
                }
              }
              if(megaTag1PoseEstimator.tagCount == 0)
              {
                doRejectUpdate = true;
              }
            }
        }

        else if (useMegaTag1 == false & megaTag2PoseEstimator != null)
        {
            
        LimelightHelpers.SetRobotOrientation(limelightName, Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate megaTag2PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

          if(Math.abs(Swerve.gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdate = true;
          }
          if(megaTag2PoseEstimator.tagCount == 0)
          {
            doRejectUpdate = true;
          }
        }
        
        else{
            doRejectUpdate = false;
        }
    
        return doRejectUpdate;
    }

    /**
     * Updates mega tag 1 and 2 variables with vision measurments. 
     * This should be called every loop.
     */
    public void updatePoseEstimates(){
        megaTag1PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        megaTag2PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        updatePoseEstimateVariables();
        if(LimelightHelpers.getBotPose2d_wpiBlue(limelightName) != null){
        }

    }

    /**
     * Updates mega tag 1 and 2 Pose2d and Timestamp variables
     */
    private void updatePoseEstimateVariables(){
        if(!rejectUpdate()){
            //TODO megaTag1PoseEstimator always null
            if(useMegaTag1 & megaTag1PoseEstimator != null){
                megaTag1Pose2d = megaTag1PoseEstimator.pose;
                megaTag1Timestamp = megaTag1PoseEstimator.timestampSeconds;
            }
            else if(!useMegaTag1 & megaTag2PoseEstimator != null){
                megaTag2Pose2d = megaTag2PoseEstimator.pose;
                megaTag2Timestamp = megaTag2PoseEstimator.timestampSeconds;
            }
        }
    }

    /**
     * @return The vision pose measurments taken from the limelight.
     */
    public Pose2d getPoseEstimate(){
        if(useMegaTag1 == true){
            return megaTag1Pose2d;
        }
        else{
            return megaTag2Pose2d;
        }
    }

    /**
     * @return The time stamp taken from the limelight.
     */
    public double getTimestamp(){
        if(useMegaTag1 == true){
            return megaTag1Timestamp;
        }
        else{
            return megaTag2Timestamp;
        }
    }


    // public Matrix<N3,N1> proportionalStdDev(){
    //     Translation2d tagTranslation2d = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation().toTranslation2d();
    //     double tagDistance = tagTranslation2d.getNorm();
    //     Matrix<N3,N1> proportionalStdDev = VecBuilder.fill(tagDistance * 1, tagDistance * 1, 99999999);
    //     return proportionalStdDev;
    // }
    public double getTagDistance(){
        Translation3d tagTranslation3d = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation();
        double tagDistance = tagTranslation3d.getNorm();
        Matrix<N3,N1> proportionalStdDev = VecBuilder.fill(tagDistance * 1, tagDistance * 1, 99999999);
        return tagDistance;
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// YOU DON'T NEED TO TOUCH THIS FOR COMMAND BASED

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Swerve;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  public Swerve s_Swerve;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    this.s_Swerve = m_robotContainer.s_Swerve;


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    /* sets vision StdDev to trusst the vision measurement and set the robot's start pose to its acctual pose
     * this should be set to trust odometry more during teleop and auto
    */
    s_Swerve.leftLimelight.setVisionMeasurementStdDevs(LimelightConstants.MEGA_TAG_1_DISABLED_STD_DEV);
    s_Swerve.rightLimelight.setVisionMeasurementStdDevs(LimelightConstants.MEGA_TAG_1_DISABLED_STD_DEV);
    /* sets the heading of the robot to its acctual heading instead of defaulting to 0 on boot up 
     * this should only be done in disabled to not effect teleop or auto control
    */
    s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), LimelightHelpers.getBotPose2d_wpiBlue("limelight-left"));
    s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), Swerve.m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    /* sets the vision StdDev values and switches to mega tag 2
     * this should also be called in teleopInit
     */
    s_Swerve.leftLimelight.setVisionMeasurementStdDevs(LimelightConstants.MEGA_TAG_1_STD_DEV);
    s_Swerve.leftLimelight.setMegaTagMode(false);
    s_Swerve.rightLimelight.setVisionMeasurementStdDevs(LimelightConstants.MEGA_TAG_1_STD_DEV);
    s_Swerve.rightLimelight.setMegaTagMode(false);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    /* sets the vision StdDev values and switches to mega tag 2
     * this should also be called in autonomousInit
     */
    s_Swerve.leftLimelight.setVisionMeasurementStdDevs(LimelightConstants.MEGA_TAG_1_STD_DEV);
    s_Swerve.leftLimelight.setMegaTagMode(false);
    s_Swerve.rightLimelight.setVisionMeasurementStdDevs(LimelightConstants.MEGA_TAG_1_STD_DEV);
    s_Swerve.rightLimelight.setMegaTagMode(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /* can set vision StdDev to be porportional to distance if needed
     * s_Swerve.setStdDev(null);
     */
     
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

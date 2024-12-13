package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final double MAX_PID_OUTPUT = 1;

    
    
    public static final class LimelightConstants {
        public static final int[] VALID_TAG_ID = {6, 7, 8, 15};

        public static final double AIM_KP = 0.0007;
        public static final double RANGE_KP = 0.004;

        public static final double SPEAKER_TY = 0; //17

        public static final Matrix<N3,N1> MEGA_TAG_1_DISABLED_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.00001);
        public static final Matrix<N3,N1> MEGA_TAG_2_DISABLED_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.00001);
        public static final Matrix<N3,N1> MEGA_TAG_1_STD_DEV = VecBuilder.fill(1, 1, 99999999);


        /* linear constants */
        public static final double lA = -0.00249975;
        public static final double lB = 0.450592;

        // public static final double lA = -0.00212533;
        // public static final double lB = 0.455131;

        /* quadratic constants */
        public static final double qA = 0.0000189893;
        public static final double qB = -0.00164208;
        public static final double qC = 0.455605;

        // public static final double qA = 0.0000125614;
        // public static final double qB = -0.0017907;
        // public static final double qC = 0.455563;

        // // Increase these numbers to trust your model's state estimates less.
        // public static final double kPositionStdDevX = 0.1;
        // public static final double kPositionStdDevY = 0.1;
        // public static final double kPositionStdDevTheta = 10;
    
        // // Increase these numbers to trust global measurements from vision less.
        // public static final double kVisionStdDevX = 5;
        // public static final double kVisionStdDevY = 5;
        // public static final double kVisionStdDevTheta = 500;
    }

    public static final class ArmConstants {

        public static final int ARM_LEAD_ID = 12;
        public static final int ARM_FOLLOW_ID = 11;
        public static final int ARM_ENCODER_ID = 4;

        public static final double MANUAL_SPEED = 1;
        public static final double MAX_PID_OUTPUT = 1;
        public static final double SLOW_PID_OUTPUT = 0.2;

        public static final double INFEED_POS = 0.598; //**0.6**

        public static final double AMP_POSE = 0.38; //0.53 werid amp
        public static final double AMP_TWO_POS = 0.31;
        
        public static final double SPEAKER_POS = INFEED_POS;
        // public static final double FARSHOT_POS;
        public static final double SHUTTLE_POS = 0.47;
        public static final double STAGE_SHUTTLE_POS = SPEAKER_POS;
        public static final double INVERSE_POS = 0.50;

        public static final double CLIMBER_POS = 0.31;

        public static final double COMP_POS = 0.55;
        public static final double DRIVE_POS = INFEED_POS -0.03;

        public static final double START_POS = 0.34;
        public static final double START_POS_2 = 0.35;

    }


    public static final class WristConstants {

        public static final int WRIST_MOTOR_ID = 15;
        public static final int WRIST_ENCODER_ID = 6;

        public static final double MANUAL_SPEED = 0.25;
        public static final double MAX_PID_OUTPUT = 0.9;
        public static final double SLOW_PID_OUTPUT = 0.1;
        public static final double SHUTTLE_PID_OUTPUT = 0.15;

        public static final double START_POS = 0.21;
        public static final double START_POS_2 = 0.20;
        public static final double START_POS_3 = 0.532;
        
        public static final double HIGH_SHOT = 0.28;
        public static final double INFEED_POS = 0.427; //0.44 //0.435

        public static final double AMP_POS = 0.32; //0.74 werid amp
        public static final double AMP_TWO_POS = 0.41;

        public static final double SPEAKER_POS = 0.45;//0.46
        public static final double INFEED_SPEAKER_POS = 0.46;
        public static final double FARSHOT_POS = 0.50;//0.52
        public static final double INVERSE_POS = 0.17;//wrist variable

        public static final double SHUTTLE_POS = 0.51;
        public static final double STAGE_SHUTTLE_POS = 0.48;
        
        public static final double MID_NOTE_POS = 0.516; //0.519
        public static final double CLOSE_MID_NOTE_POS = 0.492; //0.495
        public static final double LEFT_NOTE_POS = 0.51; //0.52
        public static final double RIGHT_NOTE_POS = 0.485; //0.538
        public static final double SPEAKER_MID_NOTE_POS = 0.49;//0.5
        
        public static final double AUTO_FARSHOT_POS = 0.526;//0.55 0.535
        public static final double AUTO_FARSHOT_POS_2 = 0.525;//0.52

        public static final double WING_POS1 = 0.542;//0.542
        public static final double WING_POS2 = 0.54;//0.542
        public static final double AMP_FARSHOT_POS = 0.535;//0.54

        public static final double SOURCE_FARSHOT_POS = 0.49; // 0.5 //0.522 //0.51
        public static final double SOURCE_FARSHOT_POS_2 = 0.53; //0.528 //0.54 //0.535

        public static final double CLIMBING_POS = 0.61;
    
        public static final double COMP_POS = 0.75;//.75--.63 for test
        public static final double DRIVE_POS = INFEED_POS;

        public static final double TEST_SHOT = 0.495;

    }

    public static final class ClimberConstants {

        public static final int CLIMBER_LEAD_ID = 14;
        public static final int CLIMBER_FOLLOW_ID = 13;
        public static final int CLIMBER_ENCODER_ID = 5;

        public static final double MANUAL_SPEED = 1;
        public static final double MAX_PID_OUTPUT = 0.75;
        public static final double SLOW_PID_OUTPUT = 0.1;

        public static final double START_POS = 1; //wrong
        public static final double TOP_POS = 1.69; //wrong
        public static final double CLIMB_POS = 4.29; //wrong

    }

    public static final class InfeedConstants {
        public static final int INFEED_LEAD_MOTOR_ID = 17;
        public static final int INFEED_FOLLOW_MOTOR_ID = 16;

        public static final double INFEED_SPEED = -1;
        public static final double POOF_SPEED = -1;
        public static final double PASS_OFF = -1;
        public static final double AMP = .6;//.6
        public static final double AMP_TWO = -0.5;
        public static final double OUTFEED = .5;
        public static final double TRAP = 0.25;//.5 for trap

    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_1_ID = 18;
        public static final int SHOOTER_MOTOR_2_ID = 19;
        //Circumference of wheel in meters, input diameter
        public static final double SHOOTER_WHEEL = Math.PI * Units.inchesToMeters(3);
        public static final double SHOOTER_GEAR_RATIO = 1;

        public static final double SHOOT_DELAY = 0.5;

        //all shooter speeds should be in max volt percent
        public static final double SPEAKER = 1; //1 changed 3/4 test w harrier
        public static final double FAR_SHOT = 1; //1
        public static final double SHUTTLE = 1; //1
        public static final double STAGE_SHUTTLE = 0.7; //.75

        public static final double POOF_SPEED = 0.3; //0.3
        public static final double POOF_SPEED_2 = 0.5;
        
        public static final double START_SHOT = 0.7;
        public static final double MID_SHOT = 1;
        public static final double LEFT_SHOT = 0.8;
        public static final double RIGHT_SHOT = 0.8;

        public static final double CENTER_SHOT = 0.35;

        public static final double AMP_TWO = 0.5;
    }

    public static final class Swerve {
        public static final int pigeonID = 10;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); 
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        //dont change this
        public static final Translation2d mod0Offset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d mod1Offset = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d mod2Offset = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d mod3Offset = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-132.297 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-58.678 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(107.139 + 180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8.173 + 180);//4.395
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0), // Translation constants //3.5
            new PIDConstants(2.5, 0, 0), // Rotation constants P = 1.5
            Swerve.maxSpeed, 
            Swerve.mod0Offset.getNorm(), // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
        );


        // DON'T NEED THIS
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

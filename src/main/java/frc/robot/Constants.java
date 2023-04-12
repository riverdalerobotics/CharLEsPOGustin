//Swerve Constants
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ArmConstants{
        public static final int kLimitSwitchPort = 9; 
        public static final int kPivotMotorID = 7;
        public static final int kExtensionMotorID = 11; 

        public static final boolean kPivotMotorInverted = true;
        public static final boolean kExtensionMotorInverted = false;

        public static final int kPivotCANCoderID = 5;
        public static final boolean kPivotCANCoderReversed = true;
        public static final double kPivotCANCoderAbsOffset = 2.322*180/Math.PI; //2.55*180/Math.PI;

        public static final double kArmDeadband = 0.1;
        public static final int kClawSolenoidForwardChannel = 0;
        public static final int kClawSolenoidReverseChannel = 1;
        public static final int kWristSolenoidForwardChannel = 2;
        public static final int kWristSolenoidReverseChannel = 3;

        public static final double kNeoCPR = 42;
       
        public static final double kPivotMotorGearRatio = (1.0/29.4)/5.0; 
        public static final double kPivotEncoderRot2Rad = kPivotMotorGearRatio * 2 * Math.PI;
        public static final double kPivotEncoderRPM2RadPerSec = kPivotEncoderRot2Rad / 60;
       
        
        public static final double kExtensionMotorGearRatio = 12.0/(10.0*44.0);
        public static final double kDiameterSpool = 36.55/1000;

        public static final double kExtensionMotorRot2Rad = kExtensionMotorGearRatio * Math.PI * kDiameterSpool;
        public static final boolean kExtensionMotorReversed = false; //TODO:determine if reverse or not

        //TODO: figure out a good kP, kI, kD for the pivot
        public static final double kPPivot = 6/(100*Math.PI/180);
        public static final double kIPivot = 0.03;//0.06;//0.36 / Math.PI;
        public static final double kDPivot = 0.01;//0.25;//0.1;

        //TODO: figure out kP, kI, kD for extension arm
        public static final double kPExtension = 4.8;
        public static final double kIExtension = 0;
        public static final double kDExtension = 0;

        public static final class PIDControlConstants{
            //TODO: find best angle
            public static final double kMidConeAngle = 1.188;
            public static final double kMidCubeAngle = 1.188;
            public static final double kBottomHybridAngle = 30*Math.PI/180;
            public static final double kTopConeAngle = 90*Math.PI/180;
            public static final double kTopCubeAngle = 90*Math.PI/180;
            //TODO: find best extension 
            public static final double kMidConeExtension = 0;
            public static final double kMidCubeExtension = 0;
            public static final double kBottomHybridExtension = 0;
            public static final double kTopConeExtension = 0.602;
            public static final double kTopCubeExtension = 0.602;

            //TODO: find best position for grabbing
            public static final double kLowGrabExtension = 0.14;
            public static final double kHighGrabExtension = 0;
            public static final double kLowGrabAngle = 0*Math.PI/180;
            public static final double kHighGrabAngle = 87*Math.PI/180;

            //TODO: find good tolerance
            public static final double kAngleTolerance = 0.05;
            public static final double kAngleToleranceTopScore = 0.1;
            public static final double kExtensionTolerance = 0.05;

            public static final double kPIDExtensionThreshold = 0.7;

        }

        public static final double kExtensionSpeedLimit = 0.7; //TODO: ask drivers what speed limit is best
        public static final double kThetaSpeedLimit = 0.7;

    }
    public static final class TurnDirectionsConstants{
        //TODO: fill out auto turn data
        //PID Constants: See "Ziegler Nichols Method" on https://youtu.be/UOuRx9Ujsog
        public static final double kC = 0d; //The kP that causes a stable osscilliation(swinging)
        public static final double pC = 0d; // The time it takes for a full osscillation at kC, unit same as periods
        //public static final double kP = 0.6*kC; //Proportional Constant
        //public static final double kI = 2*kP/pC; //Integral Constant
        //public static final double kD = kP*pC/8; //Derivative Constant
        public static final double kP = 1.5/180;
        public static final double kI = 0.2/180;
        public static final double kD = 0/180;
        public static final double POSITION_TOLERANCE = 1.0; // Acceptable error in angle for 
        //TODO: I changed from 3
        public static final double[] SETPOINTS = {-90, 0, 90, 180};
        
        //TODO: Find kC, pC, and position tolerance
    }
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kDriveMotorGearRatio = 240.0/1600.0; 
        //public static final double kTurningMotorGearRatio = 40.0/48.0; 
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kPTurning = 0.5;
        public static final double kCANCoderCPR = 4096;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22); 
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 10; 
        public static final int kBackLeftDriveMotorPort = 23; 
        public static final int kFrontRightDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 50;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 51;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 17;


        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3; 
        public static final int kBackLeftDriveAbsoluteEncoderPort = 4; 
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2; 
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
        //left in, right out
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.262463636696339;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.695352;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.755;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.2868;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //determine experimentally later
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; //determine experimentally later

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.70; //drive 70% max speed
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //determine experimentally later
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2; //original /4
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2; //original /10
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final double kPThetaController = 1.8;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        // TODO: Need to find distances for all trajectories/auto vectors
        public static final class AutoVectors {
            public static final class LeftStarting {
                public static final double[] trajectoryTo1stCone = { 0d, 0d };
                public static final double[] trajectoryTo1stDeposit = { 0d, 0d};
                public static final double[] trajectoryTo2ndCone = { 0d, 0d };
                public static final double[] trajectoryTo2ndDeposit2 = { 0d, 0d, 0d };
                public static final double trajectoryToDockDepositedCubeAndHasCone = 0d;
                public static final double trajectoryToDock2PiecesDeposited = 0d;

            }

            public static final double trajectoryTo2ndDeposit1 = 0d;

            public static final class RightStarting {
                public static final double[] trajectoryTo1stCone = { 0d, 0d };
                public static final double[] trajectoryTo1stDeposit = { 0d, 0d};
                public static final double[] trajectoryTo2ndCone = { 0d, 0d };
                public static final double[] trajectoryTo2ndDeposit2 = { 0d, 0d, 0d };
                public static final double trajectoryToDockDepositedCubeAndHasCone = 0d;
                public static final double trajectoryToDock2PiecesDeposited = 0d;
            }
        }
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kArmControllerPort = 1;
        public static final double kDeadband = 0.1;
    }

    public static final class IntakeConstants{
        public static final int kIntakeMotor1Port = 0; //TODO: find this: INTAKE MOTOR PORTs
        public static final int kIntakeMotor2Port = 0; 
        public static final double desiredIntakeSpeed = 1; //TODO: pick a proper speed
    }
    public static final class VisionConstants{
        public static final double kXCoordinate = 0; //TODO: find coordinates
        public static final double kYCoordinate = 0; // TODO: find coordinates
        public static final double kPX = 0.4; //TODO: find good PID constants for vision
        public static final double kIX = 0.5;
        public static final double kDX = 0.1;
        public static final double kPY = 0.4;
        public static final double kIY = 0.5;
        public static final double kDY = 0.1;
        public static final double kXTolerance = 0.05; //TODO: find good tolerance
        public static final double kYTolerance = 0.05;

        //Z is irrelevant
    }
    public static final class BalanceConstants{
        public static final double kP = 0.015; //TODO: find PID constants
        public static final double kI = 0;
        public static final double kD = 0;

    }
}

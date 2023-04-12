package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class AutoUtility {
  public final static TrajectoryConfig autoTrajectoryConfig =  new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kDriveKinematics);

    public final static TrajectoryConfig autoTrajectoryConfig2 =  new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared).setReversed(true)
      .setKinematics(DriveConstants.kDriveKinematics);
  
  public static Command trajectoryCommandMaker(SwerveChassisSubsystem chassis, Trajectory trajectory){
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand pickUp = new SwerveControllerCommand(
            trajectory,
            chassis::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            chassis::setModuleStates,
            chassis
    );
    

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
            new InstantCommand(() -> chassis.resetOdometry(trajectory.getInitialPose())),
            pickUp, 
            new InstantCommand(() -> chassis.stopModules())
    );
  }
}

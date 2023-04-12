// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class LimelightEnsureStartingPositionCMD extends CommandBase {
  //TODO: set up limelight properly (for April Tags)
  private final Limelight limelight;
  private final SwerveChassisSubsystem swerveSubsystem;
  private PIDController xController;
  private PIDController yController;
    /** Creates a new LimelightEnsureStartingPositionCMD. */
  public LimelightEnsureStartingPositionCMD(Limelight limelight, SwerveChassisSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(this.swerveSubsystem);
    xController = new PIDController(VisionConstants.kPX, VisionConstants.kIX, VisionConstants.kDX);
    yController = new PIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(VisionConstants.kXCoordinate);
    yController.setSetpoint(VisionConstants.kYCoordinate);
    xController.setTolerance(VisionConstants.kXTolerance);
    yController.setTolerance(VisionConstants.kYTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] pose = limelight.getBotPose();
    swerveSubsystem.driveSwerve(xController.calculate(pose[0]), yController.calculate(pose[1]), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    System.out.println("At desired starting location =)");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}

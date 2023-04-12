// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurnDirectionsConstants;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class AutoRotateRobotToAngleCMD extends CommandBase {
  private SwerveChassisSubsystem swerveChassisSubsystem;
  private PIDController thetaController;
  private double desiredAngle;
  /** Creates a new RotateRobotToAngleCMD. */
  public AutoRotateRobotToAngleCMD(SwerveChassisSubsystem swerveChassisSubsystem, double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveChassisSubsystem = swerveChassisSubsystem;
    this.desiredAngle = desiredAngle;
    thetaController = new PIDController(TurnDirectionsConstants.kP, TurnDirectionsConstants.kI, TurnDirectionsConstants.kD);
    addRequirements(this.swerveChassisSubsystem);
  }
  /**
   * This class faces the robot to one of the preset directions relative to the
   * driver
   * 
   * @param swerveChassisSubsystem this is the Chassis class
   * @param direction              this is the direction relative to the driver.
   *                               Facing towards the driver is S, Opposite is N,
   *                               to the Right is E, to the Left is W
   */
  public AutoRotateRobotToAngleCMD(SwerveChassisSubsystem swerveChassisSubsystem, char direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveChassisSubsystem = swerveChassisSubsystem;
    thetaController = new PIDController(TurnDirectionsConstants.kP, TurnDirectionsConstants.kI,
        TurnDirectionsConstants.kD);

    switch(direction){
      case 'S':
        this.desiredAngle = 179.9;
        break;
      case 'N':
        this.desiredAngle = 0;
        break;
      case 'W':
        this.desiredAngle = -90;
        break;
      case 'E':
        this.desiredAngle = 90;
    }
    addRequirements(this.swerveChassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-180, 180);
    thetaController.setSetpoint(desiredAngle);
    thetaController.setTolerance(TurnDirectionsConstants.POSITION_TOLERANCE);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveChassisSubsystem.driveSwerve(0, 0, thetaController.calculate(swerveChassisSubsystem.getHeading()), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("J'ai fini");
    thetaController.reset();
    swerveChassisSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}

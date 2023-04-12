// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class AutoPIDBalance extends CommandBase {
  /** Creates a new AutoPIDBalance. */
  private final SwerveChassisSubsystem swerveSubsystem;
  boolean maintainBalance;
  double theta;
  PIDController pitchController;

  double maxAbsStaticFrictionAngle = 0; //This is in degrees //TODO: Determine experimentally later 
  /** Whether or not we are entering backwards or forwards */
  boolean goingForwards;
  boolean doneBalancing = false;
  /** The speed to enter the charging station at */
  final double approachingSpeed = 0.5;
 /** The angle to reach in order to be considered "on" the charge station */
  final double targetClimbAngle = 11;
  public AutoPIDBalance(SwerveChassisSubsystem swerveSubsystem, boolean goingForwards) {
    this.swerveSubsystem = swerveSubsystem;
    maintainBalance = false;
    this.goingForwards = goingForwards;
    pitchController = new PIDController(BalanceConstants.kP, BalanceConstants.kI, BalanceConstants.kD);

    addRequirements(this.swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Theta roll", theta);
    
    theta = swerveSubsystem.getRollDeg();
    if(maintainBalance){
      if(!(theta<maxAbsStaticFrictionAngle) && !doneBalancing){
        swerveSubsystem.driveSwerve(pitchController.calculate(theta),0,0, false);
      }
      else{
        doneBalancing = true;
        swerveSubsystem.stopModules();
      }
    }
    else{
      System.out.println("WATCH OUT, attempting to climb station now (will move fast - ish)");
      
      if(Math.abs(theta) < targetClimbAngle){
        if(goingForwards){
          swerveSubsystem.driveSwerve(approachingSpeed, 0, 0, false);
        }
        else{
          swerveSubsystem.driveSwerve(-approachingSpeed, 0, 0, false);
        }
        
      }
      else{
        swerveSubsystem.stopModules();
        maintainBalance = true;
      }
      
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

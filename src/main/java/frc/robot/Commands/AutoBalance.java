package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class AutoBalance extends CommandBase {
  SwerveChassisSubsystem SWERVE_SUBSYSTEM;
  boolean maintainBalance;
  double theta;
  double maxAbsStaticFrictionAngle = 7; //This is in degrees //TODO: Determine experimentally later 
  double prevTheta;
  double howFar = 2.05; //TODO: find!!!! NOW
  /** Whether or not we are entering backwards or forwards */
  PIDController pitchController;
  boolean goingForwards;
  /** The speed to perform balancing at */
  final double balanceSpeed = 0.12;
  /** The speed to enter the charging station at */
  final double approachingSpeed = 0.5;
 /** The angle to reach in order to be considered "on" the charge station */
  final double targetClimbAngle = 12;
  
  /** Creates a new AutoBalance. 
   * @param swerveChassisSubsystem the swerve subsystem to use for balancing on the charge station
  */
  public AutoBalance(SwerveChassisSubsystem swerveChassisSubsystem, boolean goingForwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SWERVE_SUBSYSTEM = swerveChassisSubsystem;
    maintainBalance = false;
    theta = SWERVE_SUBSYSTEM.getRollDeg();
    this.goingForwards = goingForwards;
    pitchController = new PIDController(0.022, 0, 0);
    pitchController.setSetpoint(0);
    addRequirements(SWERVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SWERVE_SUBSYSTEM.resetModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Theta roll", theta);
    
    //theta = SWERVE_SUBSYSTEM.getRollDeg();
   
    if(maintainBalance){
      if(!(Math.abs(theta)<maxAbsStaticFrictionAngle)){
        SWERVE_SUBSYSTEM.driveSwerve(pitchController.calculate(-theta), 0, 0, false);
        /* 
        if(theta < 0){
          //positive angle
          SWERVE_SUBSYSTEM.driveSwerve(balanceSpeed, 0, 0, false);
        }
        else{
          //negative angle
          SWERVE_SUBSYSTEM.driveSwerve(-balanceSpeed, 0, 0, false);
        }*/
      }
      else{
        SWERVE_SUBSYSTEM.stopModules();
      }
    }
    else{
      System.out.println("WATCH OUT, attempting to climb station now (will move fast - ish)");
      
      if(Math.abs(SWERVE_SUBSYSTEM.getDistance()) < Math.abs(howFar)){
        if(goingForwards){
          SWERVE_SUBSYSTEM.driveSwerve(approachingSpeed, 0, 0, false);
        }
        else{
          SWERVE_SUBSYSTEM.driveSwerve(-approachingSpeed, 0, 0, false);
        }
        
      }
      else{
        SWERVE_SUBSYSTEM.stopModules();
        maintainBalance = true;
      }
      
      
    }
    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SWERVE_SUBSYSTEM.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

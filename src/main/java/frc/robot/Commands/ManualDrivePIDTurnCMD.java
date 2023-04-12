// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurnDirectionsConstants;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class ManualDrivePIDTurnCMD extends CommandBase {
  private final SwerveChassisSubsystem CHASSIS;
  private final PIDController turnController;
  private final Supplier<Double> xSpdFunction, ySpdFunction;
 
  private final Supplier<Boolean> toggleSlowModeFunction;
  private final double desiredAngle;

  public ManualDrivePIDTurnCMD(SwerveChassisSubsystem chassis, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Boolean> toggleSlow, double desiredAngle) {
    this.CHASSIS = chassis;
    this.desiredAngle = desiredAngle;
 
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.toggleSlowModeFunction = toggleSlow;

    this.turnController = new PIDController(TurnDirectionsConstants.kP, TurnDirectionsConstants.kI, TurnDirectionsConstants.kD);

    addRequirements(chassis);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setSetpoint(desiredAngle);
    turnController.setTolerance(TurnDirectionsConstants.POSITION_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(toggleSlowModeFunction.get()){
      CHASSIS.toggleSpeedMode();
    }

  
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();


    CHASSIS.driveSwerve(xSpeed, ySpeed, turnController.calculate(CHASSIS.getHeading()), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnController.reset();
    CHASSIS.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return turnController.atSetpoint();
    return false;
  }
}

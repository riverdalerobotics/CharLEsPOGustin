// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

public class AutoMoveArmToDesiredPositionCMD extends CommandBase {
  private ArmSubsystem armSubsystem;
  private PIDController thetaController;
  private double desiredAngle;
  private double desiredExtension;
  private PIDController extensionController;

 
    /** Creates a new AutoMoveArmToDesiredPosition. */
  public AutoMoveArmToDesiredPositionCMD(ArmSubsystem armSubsystem, double desiredAngle, double desiredExtension) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);
    thetaController = new PIDController(ArmConstants.kPPivot, ArmConstants.kIPivot, ArmConstants.kDPivot);
    extensionController = new PIDController(ArmConstants.kPExtension, ArmConstants.kIExtension, ArmConstants.kDExtension);
    this.desiredAngle = desiredAngle;
    this.desiredExtension = desiredExtension;
   
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.setSetpoint(desiredAngle);
    extensionController.setSetpoint(desiredExtension);
    thetaController.setTolerance(ArmConstants.PIDControlConstants.kAngleTolerance);
    extensionController.setTolerance(ArmConstants.PIDControlConstants.kExtensionTolerance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    armSubsystem.rotateArm(thetaController.calculate(armSubsystem.getPivotAngle()));
    if(thetaController.atSetpoint() || armSubsystem.getPivotAngle() > 0.5 || desiredAngle < armSubsystem.getPivotAngle()){
      armSubsystem.extendArm(extensionController.calculate(armSubsystem.getExtensionMetres()));
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopAll();
    thetaController.reset();
    extensionController.reset();
    System.out.println("Completed arm stuff");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return thetaController.atSetpoint() && extensionController.atSetpoint();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSubsystem;

public class RetractArmCommand extends CommandBase {
  /** Creates a new RetractArmCommand. */
  final double retractionSpeed = -0.5;
  final double fastRetractionSpeed = -1;
  ArmSubsystem armSubsystem;
  public RetractArmCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armSubsystem.getExtensionMetres() > 0.1){
      armSubsystem.extendArm(fastRetractionSpeed);
    }
    else{
      armSubsystem.extendArm(retractionSpeed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopExtending();
    armSubsystem.resetExtensionEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.getLimitSwitchValue();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase {
  /** Creates a new IntakeDefaultCommand. */
  private IntakeSubsystem intakeSubsystem;
  private Supplier<Boolean> intakeFunction;
  private Supplier<Boolean> outtakeFunction;
  private double speed;
  public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> intakeFunct, Supplier<Boolean> outtakeFunct, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    this.intakeFunction = intakeFunct;
    this.outtakeFunction = outtakeFunct;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeFunction.get()){
      intakeSubsystem.intake(speed);
    }
    else if(outtakeFunction.get()){
      intakeSubsystem.outtake(speed);
    }
    else{
      intakeSubsystem.stop();
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

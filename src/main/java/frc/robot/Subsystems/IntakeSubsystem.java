// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  WPI_TalonSRX intakeMotor1;
  WPI_TalonSRX intakeMotor2;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor1 = new WPI_TalonSRX(IntakeConstants.kIntakeMotor1Port);
    intakeMotor2 = new WPI_TalonSRX(IntakeConstants.kIntakeMotor2Port);

  }
  public void intake(double speed){
    intakeMotor1.set(speed);
    intakeMotor2.set(-speed);
  }
  public void outtake(double speed){
    intakeMotor1.set(-speed);
    intakeMotor2.set(speed);
  }
  public void stop(){
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

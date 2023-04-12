// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ClawSubsystem extends SubsystemBase {
  DoubleSolenoid wristPiston;
  DoubleSolenoid clawPiston;
  Value wristState;
  Value clawState;
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    wristPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.kWristSolenoidForwardChannel, ArmConstants.kWristSolenoidReverseChannel);
    clawPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.kClawSolenoidForwardChannel, ArmConstants.kClawSolenoidReverseChannel);
    wristState = Value.kOff;
    clawState = Value.kOff;
  }
  public void openClaw(){
    clawState = Value.kReverse;
    clawPiston.set(clawState);
  }
  public void closeClaw(){
    clawState = Value.kForward;
    clawPiston.set(clawState);
  }
  public void wristUp(){
    wristState = Value.kForward;
    wristPiston.set(wristState);
  }
  public void wristDown(){
    wristState = Value.kReverse;
    wristPiston.set(wristState);
  }
  public void toggleWrist(){
    wristState = (wristState == Value.kForward || wristState == Value.kOff) ? Value.kReverse : Value.kForward;
    wristPiston.set(wristState);
  }
  public void toggleClaw(){
    clawState = (clawState == Value.kForward || clawState == Value.kOff) ? Value.kReverse : Value.kForward;
    clawPiston.set(clawState);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

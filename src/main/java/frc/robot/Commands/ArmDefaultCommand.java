// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants.PIDControlConstants;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmSubsystem armSubsystem;
  private Supplier<Double> armRotationFunct;
  private Supplier<Double> armExtensionFunct;
  private Supplier<Boolean> grabLowFunction, grabHighFunction;
  private Supplier<Boolean> enableManualExtensionFunct, enableManualRotationFunct, midLevelFunct, topLevelFunct, bottomLevelFunct;
  private boolean cubeMode;
  private PIDController thetaPivotController; 
  private PIDController extensionController;
  private boolean PIDModeExtension;
  private boolean PIDModePivot;
  private double PIDExtensionThreshold;
  //allows for simultaneous PID and manual control
  public ArmDefaultCommand(ArmSubsystem armSubsystem, Supplier<Double> armRotFunction, Supplier<Double> armExtFunction,  Supplier<Boolean> enableManualExtensionFunct, Supplier<Boolean> enableManualRotationFunct, Supplier<Boolean> midLevelFunct, 
    Supplier<Boolean> topLevelFunct, Supplier<Boolean> bottomLevelFunct, Supplier<Boolean> grabLowFunct, Supplier<Boolean> grabHighFunct) {

    this.armSubsystem = armSubsystem;
    this.armRotationFunct = armRotFunction;
    this.armExtensionFunct = armExtFunction;
    
    this.enableManualExtensionFunct = enableManualExtensionFunct;
    this.enableManualRotationFunct = enableManualRotationFunct;
    this.midLevelFunct = midLevelFunct;
    this.topLevelFunct = topLevelFunct;
    this.bottomLevelFunct = bottomLevelFunct;
    this.grabHighFunction = grabHighFunct;
    this.grabLowFunction = grabLowFunct;
    this.PIDExtensionThreshold = PIDControlConstants. kPIDExtensionThreshold;
    addRequirements(this.armSubsystem);

    thetaPivotController = new PIDController(ArmConstants.kPPivot, ArmConstants.kIPivot, ArmConstants.kDPivot);
    thetaPivotController.setTolerance(ArmConstants.PIDControlConstants.kAngleTolerance);
    extensionController =  new PIDController(ArmConstants.kPExtension, ArmConstants.kIExtension, ArmConstants.kDExtension);
    PIDModePivot = false;
    cubeMode = false;
    PIDModeExtension = false;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialisePID(armSubsystem.getPivotAngle(), armSubsystem.getExtensionMetres());
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(enableManualExtensionFunct.get()){
      PIDModeExtension = false;
    }
    if(enableManualRotationFunct.get()){
      PIDModePivot = false;
    }
    
    if(bottomLevelFunct.get()){
      //Should be a pressed button (not hold)
      initialisePID(ArmConstants.PIDControlConstants.kBottomHybridAngle, ArmConstants.PIDControlConstants.kBottomHybridExtension);
    }
    else if(midLevelFunct.get()){
      if(cubeMode){
        initialisePID(ArmConstants.PIDControlConstants.kMidCubeAngle, ArmConstants.PIDControlConstants.kMidCubeExtension);
      }
      else{
        initialisePID(ArmConstants.PIDControlConstants.kMidConeAngle, ArmConstants.PIDControlConstants.kMidConeExtension);
      }
      
    }
    else if(topLevelFunct.get()){
      if(cubeMode){
        initialisePID(ArmConstants.PIDControlConstants.kTopCubeAngle, ArmConstants.PIDControlConstants.kTopCubeExtension);
      }
      else{
        initialisePID(ArmConstants.PIDControlConstants.kTopConeAngle, ArmConstants.PIDControlConstants.kTopConeExtension);
      }
    }
    else if(grabHighFunction.get()){
      initialisePID(ArmConstants.PIDControlConstants.kHighGrabAngle, ArmConstants.PIDControlConstants.kHighGrabExtension);

    }
    else if(grabLowFunction.get()){
      initialisePID(ArmConstants.PIDControlConstants.kLowGrabAngle, ArmConstants.PIDControlConstants.kLowGrabExtension);
    }
    if(PIDModeExtension && (thetaPivotController.atSetpoint() || armSubsystem.getPivotAngle() >= PIDExtensionThreshold)){
      armSubsystem.extendArm(extensionController.calculate(armSubsystem.getExtensionMetres()));
    }
    else{
      //armExtensionFunct is negative because without it, pushing
      //joystick up would retract arm
      double speedExtInput = -armExtensionFunct.get();
      if(Math.abs(speedExtInput) >= OIConstants.kDeadband){
        armSubsystem.extendArm(speedExtInput*ArmConstants.kExtensionSpeedLimit);
      }
      else{
        armSubsystem.stopExtending(); 
      }
    }
    if(PIDModePivot){
      armSubsystem.rotateArm(thetaPivotController.calculate(armSubsystem.getPivotAngle()));
    }
    else{
      double speedThetaInput = armRotationFunct.get();
      if(Math.abs(speedThetaInput) >= OIConstants.kDeadband){
        armSubsystem.rotateArm(speedThetaInput*ArmConstants.kThetaSpeedLimit);
      }
      else{
        armSubsystem.stopRotating();
      }
    }
    outputDebuggingData();
  }

  public void initialisePID(double desiredAngle, double desiredExtension){
    this.PIDModePivot = true;
    this.PIDModeExtension = true;
    
    this.thetaPivotController.setSetpoint(desiredAngle);
    this.extensionController.setSetpoint(desiredExtension);
  
    this.extensionController.reset();
    this.thetaPivotController.reset();

    //this.armSubsystem.resetExtensionEncoder();
  }
  
  public void outputDebuggingData(){
    //TODO: fill this in if necessary -probably not, J
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    thetaPivotController.reset();
    extensionController.reset();
    armSubsystem.stopAll();
    if(interrupted){
      initialisePID(armSubsystem.getPivotAngle(), armSubsystem.getExtensionMetres());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  
  CANSparkMax pivotMotor;
  WPI_CANCoder pivotEncoder;

  CANSparkMax extensionMotor;
  RelativeEncoder extensionEncoder;

  DigitalInput limitSwitch;

   /** Creates a new ArmSubsystem. Sets up Spark Max Motor Controllers, CANCoder, and relative encoder for Neo*/
  public ArmSubsystem() {
    pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorID, MotorType.kBrushless);
    pivotMotor.setInverted(ArmConstants.kPivotMotorInverted); 
    extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotorID, MotorType.kBrushless);
    //upwards is now +
    pivotEncoder = new WPI_CANCoder(ArmConstants.kPivotCANCoderID);
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.magnetOffsetDegrees = ArmConstants.kPivotCANCoderAbsOffset; //TODO: determine offset
    config.sensorDirection = ArmConstants.kPivotCANCoderReversed; 
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // fix range to + 180 to -180
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    pivotEncoder.configAllSettings(config);
    
    extensionEncoder = extensionMotor.getEncoder();
    extensionEncoder.setPositionConversionFactor(ArmConstants.kExtensionMotorRot2Rad);
    limitSwitch = new DigitalInput(ArmConstants.kLimitSwitchPort);
    
  }
  public boolean getLimitSwitchValue(){
    return !limitSwitch.get(); //lim switch is inverted 
  }
  public void extendArm(double speed){
    extensionMotor.set(speed);
  }
  public void rotateArm(double speed){
    pivotMotor.set(speed);
  }
  public double getPivotAngle(){
    return pivotEncoder.getAbsolutePosition();
  }
  public double getExtensionMetres(){
    return extensionEncoder.getPosition();
  }
  public void resetExtensionEncoder(){
    extensionEncoder.setPosition(0);
  }

  public void stopExtending(){
    extensionMotor.set(0);
  }
  public void stopRotating(){
    pivotMotor.set(0);
  }
  public void stopAll(){
    stopExtending();
    stopRotating();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("limit switch", getLimitSwitchValue());
    SmartDashboard.putNumber("piv enc", getPivotAngle());
    SmartDashboard.putNumber("ext enc", getExtensionMetres());
  }
}

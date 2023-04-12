package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.SwerveChassisSubsystem;

public class SwerveJoystickCMD extends CommandBase {

    private final SwerveChassisSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;

    private final Supplier<Boolean> gyroZeroFunction;
    private final Supplier<Boolean> toggleSlowModeFunction;
    private final Supplier<Double> speedBoost;
    private final Supplier<Double> speedDampener;
    private boolean isFieldOriented;
  
  
    public SwerveJoystickCMD(SwerveChassisSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> resetGyroZero, Supplier<Boolean> toggleSlow, Supplier<Double> speedBoost, Supplier<Double> speedDampener) {
        this.swerveSubsystem = swerveSubsystem;
        
        this.gyroZeroFunction = resetGyroZero;
        this.isFieldOriented = true;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.toggleSlowModeFunction = toggleSlow;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.speedBoost = speedBoost;
        this.speedDampener = speedDampener;
    
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Field Oriented Mode?", isFieldOriented);
        if(gyroZeroFunction.get()){
            swerveSubsystem.zeroHeading();
        }
       
        if(toggleSlowModeFunction.get()){
            swerveSubsystem.toggleSpeedMode();
        }
        if(fieldOrientedFunction.get()){
            
            isFieldOriented = (isFieldOriented)? false : true;
            
        }
        
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
       
        swerveSubsystem.driveSwerve(xSpeed, ySpeed, turningSpeed, isFieldOriented);

        double speedIncrease = speedBoost.get();
        double speedDecrease = speedDampener.get();
        swerveSubsystem.speedUp(speedIncrease, 0);
        

    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

/** Add your docs here. */
public class OperatorInput {
    XboxController movementController;
    XboxController armController;
    /**
     * Constructor for the OperatorInput class which contains a
     * variety of useful methods for getting values from the Xbox controllers.
     * There are two Xbox Controllers: one for the arm and the other for movement
     */
    public OperatorInput() {
        movementController = new XboxController(OIConstants.kDriverControllerPort);
        armController = new XboxController(OIConstants.kArmControllerPort);
    }
    /**
     * For use in SwerveJoystickCMD. 
     * This tells the robot how fast to move left or right 
     * @return X axis value of left joystick on movement controller
     */
    public double getLeftXMovement() {
        return -movementController.getLeftX();
    }
    /**
     * For use in SwerveJoystickCMD.
     * This tells the robot how fast to move forwards or backwards
     * @return Y axis value of left joystick on movement controller
     */
    public double getLeftYMovement() {
        return -movementController.getLeftY();
    }
    //Turn
    public double getRightXMovement() {
    
        return -movementController.getRightX();
    }
    
    
    //Extends and reduces telescope
    public double getLeftYArm() {
        return armController.getLeftY();
    }
    //Pivots arm
    public double getRightYArm() {
        return armController.getRightY();
    }
    
    
    public boolean toggleFieldOriented() {
        return movementController.getLeftBumperPressed();
    } 

    //zeros gyro (resets what the 0 angle is (useful for FOD))
    public boolean getStartButtonMovement() {
        return movementController.getStartButtonPressed();
    }
    
    //halves the translational and rotational speed
    public boolean slowMode(){
        return movementController.getRightBumperPressed();
    } 

    //claw and wrist control for arm controller
    public boolean openClawLeftBumperA(){
        return armController.getLeftBumperPressed();
    }
    public boolean closeClawRightBumperA(){
        return armController.getRightBumperPressed();
    }
    public boolean wristUpLeftTriggerA(){
        return armController.getLeftTriggerAxis() >= OIConstants.kDeadband;
    }
    public boolean wristDownRightTriggerA(){
        return armController.getRightTriggerAxis() >= OIConstants.kDeadband;
    }
 
    //https://pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
    //Eg of field oriented. If robot is rotate 180 degrees and left joystick Y is held forwards,
    //The robot will move forward relative to the field, not its own frame of reference
    //IE. towards the opponents side, and not towards the driver
    /* 
    public boolean toggleFieldOriented(){
        return movementController.getLeftTriggerAxis() >= OIConstants.kDeadband;
    }
    public boolean slowMode(){
        return movementController.getRightTriggerAxis() >= OIConstants.kDeadband;
    }
    */

    //cube or cone selection
    public boolean getStartButtonCubeA(){
        return armController.getStartButtonPressed();
    }
    public boolean getBackButtonConeA(){
        return armController.getBackButtonPressed();
    }
    //low or high pick up
    private final List<Integer> upAngles = List.of(0,45,315);
    public boolean getUpDPadA(){
        return upAngles.contains(armController.getPOV());
    }
    private final List<Integer> downAngles = List.of(135, 180, 225);
    public boolean getDownDPadA(){
        return downAngles.contains(armController.getPOV());
    }
    //low, high, mid score
    /**
     * Score Low (hybrid node)
     * @return A button pressed on arm controller
     */
    public boolean getAButtonA(){
        
        return armController.getAButtonPressed();
    }
    /**
     * Score mid
     * @return B button on arm controller
     */
    public boolean getBButtonA(){
        return armController.getBButtonPressed();
    }
    /**
     * score high
     * @return Y button on arm controller
     */
    public boolean getYButtonA(){
        return armController.getYButtonPressed();
    }

    //rotate robot to 4 directions
    /**
     * Turn to 0
     * @return movement controller y button state
     */
    public boolean getYButtonM(){
        return movementController.getYButton();
    }
    /**
     * Turn to 90
     * @return movement controller B button state
     */
    public boolean getBButtonM(){
        return movementController.getBButton();
    }
    /**
     * Turn to 180
     * @return movement controller A button state
     */
    public boolean getAButtonM(){
        return movementController.getAButton();
    }
    /**
     * Turn to 270 
     * @return movement controller X button state
     */
    public boolean getXButtonM(){
        
        return movementController.getXButton();
    }
    public boolean getManualExtLeftStickA(){
        return armController.getLeftStickButtonPressed();
    }
    public boolean getManualRotRightStickA(){
        return armController.getRightStickButtonPressed();
    }

    public boolean getRetractXButtonA(){
        return armController.getXButton();
    }
    public boolean getExtEnc0Button(){
        return armController.getPOV() == 90;
    }
    //Raises drive speed in teleop by up to 10%
    public double getRightTriggerM(){
        return movementController.getRightTriggerAxis();
    }

    public double getLeftTriggerM(){
        return movementController.getLeftTriggerAxis();
    }

}

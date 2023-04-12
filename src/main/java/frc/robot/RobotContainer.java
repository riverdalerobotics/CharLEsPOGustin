package frc.robot;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmDefaultCommand;
import frc.robot.Commands.AutoMoveArmToDesiredPositionCMD;
import frc.robot.Commands.Autos;
import frc.robot.Commands.IntakeDefaultCommand;
import frc.robot.Commands.ManualDrivePIDTurnCMD;
import frc.robot.Commands.RetractArmCommand;
import frc.robot.Commands.SwerveJoystickCMD;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ClawSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class RobotContainer {

    private final SwerveChassisSubsystem swerveSubsystem = new SwerveChassisSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final OperatorInput OI = new OperatorInput();
    private final Limelight limelight = new Limelight();
    
    Autos autoFactory = new Autos(swerveSubsystem, intakeSubsystem, clawSubsystem, armSubsystem, limelight);
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(
                swerveSubsystem,
                () -> OI.getLeftYMovement(),
                () -> OI.getLeftXMovement(),
                () -> OI.getRightXMovement(),
                () -> OI.toggleFieldOriented(),
                () -> OI.getStartButtonMovement(),
                () -> OI.slowMode(),
                () -> OI.getRightTriggerM(),
                () -> OI.getLeftTriggerM()
        ));
        //intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(intakeSubsystem, ()->OI.intakeLeftTriggerM(), ()->OI.outtakeRightTriggerM(), IntakeConstants.desiredIntakeSpeed));
        armSubsystem.setDefaultCommand(new ArmDefaultCommand(armSubsystem, ()->OI.getRightYArm(), ()->OI.getLeftYArm(), ()->OI.getManualExtLeftStickA(), ()->OI.getManualRotRightStickA(), ()->OI.getBButtonA(), ()->OI.getYButtonA(), ()->OI.getAButtonA(),()->OI.getDownDPadA(),()->OI.getUpDPadA()));
        configureBindings();


    }

    public void configureBindings(){
        //Buttons for the claw 
        new Trigger(()->OI.closeClawRightBumperA()).onTrue(new InstantCommand(()->clawSubsystem.closeClaw()));
        new Trigger(()->OI.openClawLeftBumperA()).onTrue(new InstantCommand(()->clawSubsystem.openClaw()));
        //buttons for the wrist
        new Trigger(()->OI.wristUpLeftTriggerA()).onTrue(new InstantCommand(()->clawSubsystem.wristUp()));
        new Trigger(()->OI.wristDownRightTriggerA()).onTrue(new InstantCommand(()->clawSubsystem.wristDown()));
        
        //buttons for rotations of robot
        new Trigger(()->OI.getYButtonM()).whileTrue(new ManualDrivePIDTurnCMD(swerveSubsystem, ()->OI.getLeftYMovement(), ()->OI.getLeftXMovement(),  ()->OI.slowMode(), 0));
        new Trigger(()->OI.getBButtonM()).whileTrue(new ManualDrivePIDTurnCMD(swerveSubsystem, ()->OI.getLeftYMovement(), ()->OI.getLeftXMovement(),  ()->OI.slowMode(), 90));
        new Trigger(()->OI.getAButtonM()).whileTrue(new ManualDrivePIDTurnCMD(swerveSubsystem, ()->OI.getLeftYMovement(), ()->OI.getLeftXMovement(),  ()->OI.slowMode(), 179.9));
        new Trigger(()->OI.getXButtonM()).whileTrue(new ManualDrivePIDTurnCMD(swerveSubsystem, ()->OI.getLeftYMovement(), ()->OI.getLeftXMovement(),  ()->OI.slowMode(), -90));

        //button for retraction of the arm (Button changed to 0 extension and rotation of arm)
        //new Trigger(()->OI.getRetractXButtonA()).onTrue(new RetractArmCommand(armSubsystem));

        //backup button for zeroing the encoders
        new Trigger(()->OI.getExtEnc0Button()).onTrue(new InstantCommand(()->armSubsystem.resetExtensionEncoder()));
    
        //TODO: Remove whatever uses this button
        new Trigger(()->OI.getRetractXButtonA()).onTrue(new SequentialCommandGroup(new RetractArmCommand(armSubsystem), new AutoMoveArmToDesiredPositionCMD(armSubsystem, 0, 0.0).until(()->OI.getStartButtonCubeA())));
    }
    public Command getTestAuto(){
        //return autoFactory.singleCubeBackOutDoNothing();
        return autoFactory.testAuto();
    }
    //TODO: add more getAutonomousCommand() methods
    public Command getTag3_8IntakeAuto(){
        return autoFactory.highCubeGrabConeTag8_3();
    }
    public Command getTag1_6IntakeAuto(){
        return autoFactory.highCubeGrabConeTag1_6();
    }
    public Command getMid2_7BalanceAuto(){
        return autoFactory.highCubeMidBackUpBalance();
    }
    public Command getTag1_6IntakePrepToScoreAuto(){
        return autoFactory.highCubeGrabConeDriveBackPrepareToScoreTag1_6();
    }
    public Command getTag3_8IntakePrepToScoreAuto(){
        return autoFactory.highCubeGrabConeDriveBackPrepareToScoreTag3_8();
    }
    public Command getTag3TwoPieceAuto(){
        return autoFactory.highCubeGrabConeDriveBackPrepareToScoreTag3_8();
    }
    public Command getTag6TwoPieceAuto(){
        return autoFactory.highCubeGrabConeDriveBackScoreHighConeTag1_6();
    }
    public Command getDropCubeDoNothing(){
        return autoFactory.dropCubeDoNothing();
    }
    public Command flipClaw(){
        return new InstantCommand(()->clawSubsystem.openClaw());
    }
}
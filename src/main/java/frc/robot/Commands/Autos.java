// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Limelight;
import frc.robot.Constants.ArmConstants;

import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ClawSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveChassisSubsystem;



import frc.robot.Constants.AutoConstants.AutoVectors;
import frc.robot.Constants.AutoConstants.AutoVectors.LeftStarting;
import frc.robot.Constants.AutoConstants.AutoVectors.RightStarting;



public final class Autos {
  /** Example factory for an autonomous command. */
  private final SwerveChassisSubsystem CHASSIS;
  private final IntakeSubsystem INTAKE;
  private final ClawSubsystem CLAW;
  private final ArmSubsystem ARM;
  private final Limelight LIME;
 

  public Autos(SwerveChassisSubsystem chassis, IntakeSubsystem intake, ClawSubsystem claw, ArmSubsystem arm, Limelight lime) {
    this.CHASSIS = chassis;
    this.INTAKE = intake;
    this.CLAW = claw;
    this.ARM = arm;
    this.LIME = lime;
  }
  public Command testAuto(){
    
    return highCubeGrabConeDriveBackPrepareToScoreTag3_8();
    //return new AutoPIDBalance(CHASSIS, false);
    //return new AutoRotateRobotToAngleCMD(CHASSIS, 179.9);
    //return scoreTopNode(true);
  }
  /** Tag 7 and 2 */
  public Command highCubeMidBackUpBalance(){
    return new SequentialCommandGroup(
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CLAW.wristUp()),
      scoreTopNode(true),
      new RetractArmCommand(ARM),
      new AutoMoveArmToDesiredPositionCMD(ARM, 0, 0),
      autoDriveToCentreOfBalance()


    );
  }
  public Command dropCubeDoNothing(){
    return new SequentialCommandGroup(
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CLAW.wristUp()),
      scoreTopNode(true),
      new RetractArmCommand(ARM),
      new AutoMoveArmToDesiredPositionCMD(ARM, 0, 0)
    );
  }
  


  /**
   * Constructs an auto to score, drive to grab a piece, and come back with arm up prepared to score in tele-op
   * @param reverse Command to reverse after scoring first piece
   * @param pushIntoCone Small adjustment left or right + forward to grab cone
   * @param driveBack Drives back after grabbing cone and rotating 180 degrees
   * @return SequentialCommandGroup
   */
  public Command buildHighCubeGrabConeDriveBack(Command reverse, Command pushIntoCone, Command driveBack) {
    return new SequentialCommandGroup(
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CLAW.wristUp()),
      scoreTopNode(true),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AutoMoveArmToDesiredPositionCMD(ARM, ArmConstants.PIDControlConstants.kLowGrabAngle, ArmConstants.PIDControlConstants.kLowGrabExtension)
        ),
        reverse
      ),
      new AutoRotateRobotToAngleCMD(CHASSIS, 179.9),
      new InstantCommand(()->CLAW.openClaw()),
      pushIntoCone,
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CHASSIS.zeroHeading()),
      new RetractArmCommand(ARM),
      new AutoRotateRobotToAngleCMD(CHASSIS, 179.9),
      new ParallelCommandGroup(
        new AutoMoveArmToDesiredPositionCMD(ARM, ArmConstants.PIDControlConstants.kMidConeAngle, ArmConstants.PIDControlConstants.kMidConeExtension),
        driveBack
      )
    ); 
  }

  public Command highCubeGrabConeDriveBackPrepareToScoreTag3_8(){
    return buildHighCubeGrabConeDriveBack(autoReverseFromStartTag3_8(), pushIntoConeFromTag3_8(), goAndPrepareToDepositFromTag3());
  }
  //Starts at AprilTag1, deposits high cube, then drives to pick up cone and comes back and gets close to cone node
  public Command highCubeGrabConeDriveBackPrepareToScoreTag1_6() {
    return buildHighCubeGrabConeDriveBack(newAutoReverseFromStartTag1_6(), newPushIntoConeFromTag1_6(), goAndPrepareToDepositFromTag1_6());
  }


  //These autos score, then drive to and grab a cone, then turn around to face the nodes and stops
  public Command highCubeGrabConeTag8_3() {
    return buildHighCubeGrabConeDriveBack(autoReverseFromStartTag3_8(), pushIntoConeFromTag3_8(), new InstantCommand());
  }
  public Command highCubeGrabConeTag1_6() {
    return buildHighCubeGrabConeDriveBack(newAutoReverseFromStartTag1_6(), newPushIntoConeFromTag1_6(), new InstantCommand());
  }




  public Command buildHighCubeGrabConeDriveBackHighCone(Command reverse, Command pushIntoCone, Command driveBack) {
    return new SequentialCommandGroup(
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CLAW.wristUp()),
      scoreTopNode(true),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AutoMoveArmToDesiredPositionCMD(ARM, ArmConstants.PIDControlConstants.kLowGrabAngle, ArmConstants.PIDControlConstants.kLowGrabExtension)
        ),
        reverse
      ),
      new AutoRotateRobotToAngleCMD(CHASSIS, 179.9),
      new InstantCommand(()->CLAW.openClaw()),
      pushIntoCone,
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CHASSIS.zeroHeading()),
      new RetractArmCommand(ARM),
      new AutoRotateRobotToAngleCMD(CHASSIS, 179.9),
      new ParallelCommandGroup(
        new AutoMoveArmToDesiredPositionCMD(ARM, ArmConstants.PIDControlConstants.kTopConeAngle, ArmConstants.PIDControlConstants.kTopConeExtension),
        driveBack
      ),
      new InstantCommand(()->CLAW.wristDown()),
      new WaitCommand(0.1),
      new InstantCommand(() ->CLAW.openClaw()) 
    ); 
  }

  //Scores high cube, drives to middle and reverse, grabs cone, drives back to cone node and scores
  public Command highCubeGrabConeDriveBackScoreHighConeTag3_8(){
    return buildHighCubeGrabConeDriveBackHighCone(autoReverseFromStartTag3_8(), pushIntoConeFromTag3_8(), goToDepositConeTag3_8());
  }
  public Command highCubeGrabConeDriveBackScoreHighConeTag1_6(){
    return buildHighCubeGrabConeDriveBackHighCone(newAutoReverseFromStartTag1_6(), newPushIntoConeFromTag1_6(), goToDepositConeTag1_6());
  }




  
  /* 
  public Command highCubeGrabConeDriveBackScoreTag6() {
    return new SequentialCommandGroup(
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CLAW.wristUp()),
      scoreTopNode(true),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AutoMoveArmToDesiredPositionCMD(ARM, ArmConstants.PIDControlConstants.kLowGrabAngle, ArmConstants.PIDControlConstants.kLowGrabExtension)
        ),
        new SequentialCommandGroup(
          autoReverseFromStartTag1_6()
        )
      ),
      new AutoRotateRobotToAngleCMD(CHASSIS, 179.9),
      new InstantCommand(()->CLAW.openClaw()),
      pushIntoConeFromTag1_6(), //1 and 6 same thi
      new InstantCommand(()->CLAW.closeClaw()),
      new InstantCommand(()->CHASSIS.zeroHeading()),
      new AutoRotateRobotToAngleCMD(CHASSIS, 179.9),
      new ParallelCommandGroup(
        goToDepositConeTag1_6(),
        new AutoMoveArmToDesiredPositionCMD(ARM, ArmConstants.PIDControlConstants.kTopConeAngle, ArmConstants.PIDControlConstants.kTopConeExtension + 0.1)
      ),
      new InstantCommand(()->CLAW.wristDown()),
      new WaitCommand(0.1),
      new InstantCommand(() ->CLAW.openClaw()) 
    ); 

  }
  */
  
  public Command newAutoReverseFromStartTag1_6(){
    //tag1 and 6
    Trajectory trajectoryToPickUp = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), //x and y are in meters
            List.of(
                    new Translation2d(-0.5, -0.1),
                    new Translation2d(-1.5, -0.1),
                    new Translation2d(-2, -0.1) 
                    ),
            new Pose2d(-4.0, -0.1, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig2);

    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPickUp);
  }

  public Command newPushIntoConeFromTag1_6(){ 
    Trajectory trajectoryToPush = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                    new Translation2d(0.25, 0) 
                    ),
            new Pose2d(1.45, 0, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPush);
  }




  
  /**
   * Scores either a cube or cone in the top node
   * 
   * @param depositingCone whether it is scoring a cone or a cube(false)
   */
  private Command scoreTopNode(boolean depositingCone){
    double angle;
    double extension;

    if (depositingCone) {
      angle = ArmConstants.PIDControlConstants.kTopConeAngle;
      extension = ArmConstants.PIDControlConstants.kTopConeExtension + 0.08;
    } else {
      angle = ArmConstants.PIDControlConstants.kTopCubeAngle;
      extension = ArmConstants.PIDControlConstants.kTopCubeExtension + 0.08;
    }

    return new SequentialCommandGroup(
        new AutoMoveArmToDesiredPositionCMD(ARM, angle, extension),
        new InstantCommand(()->CLAW.wristDown()),
        new InstantCommand(() -> {
          try {
              Thread.sleep(100);
          } catch (Exception e) {
          }}),
        new InstantCommand(() -> CLAW.openClaw()),
        new InstantCommand(() -> {
          try {
              
              Thread.sleep(250);
          } catch (Exception e) {
          }}),
        new InstantCommand(()->CLAW.wristUp()),
        new InstantCommand(() -> {
          try {
              
              Thread.sleep(200);
          } catch (Exception e) {
          }}),
        new InstantCommand(()->CLAW.closeClaw())
      );
  }
  /**T1,6 */
  public Command autoReverseFromStartTag1_6() { 
    //tag1 and 6
    Trajectory trajectoryToPickUp = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), //x and y are in meters
            List.of(
                    new Translation2d(-0.5, -0.35),
                    new Translation2d(-1.5, -0.35),
                    new Translation2d(-2, -0.35) 
                    ),
            new Pose2d(-3.8, -0.35, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig2);

    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPickUp);
  }
  
  public Command autoReverseFromStartTag3_8() { 
    Trajectory trajectoryToPickUp = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), //x and y are in meters
            List.of(
                    new Translation2d(-0.5, 0.35),
                    new Translation2d(-1.5, 0.35),
                    new Translation2d(-2, 0.35) 
                    ),
            new Pose2d(-3.8, 0.35, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig2);

    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPickUp);
  }
  
  //144.018 //65.018
  public Command autoDriveToCentreOfBalance() { 
    Trajectory trajectoryToPickUp = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), //x and y are in meters
            List.of(
                    new Translation2d(-0.5, 0),
                    new Translation2d(-1.5, 0)
                    
                    ),
            new Pose2d(-3.0, 0, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig2);

    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPickUp);
  }
  
   
  public Command pushIntoConeFromTag1_6(){ 
    Trajectory trajectoryToPush = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                    new Translation2d(0.0, -0.35),
                    new Translation2d(0.3, -0.35)
                    ),
            new Pose2d(1.25, -0.35, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPush);
  }
  public Command pushIntoConeFromTag3_8(){ 
    Trajectory trajectoryToPush = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                    new Translation2d(0.2, 0.45) //0.4
                    ),
            new Pose2d(1.15, 0.45, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToPush);
  }

 public Command goToDepositConeTag1_6(){ 
    Trajectory trajectoryToDeposit = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                  new Translation2d(1,-0.4)
                    ),
            new Pose2d(4.9, -0.4, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToDeposit);
  }
  public Command goToDepositConeTag3_8(){ 
    Trajectory trajectoryToDeposit = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                  new Translation2d(1,0.4)
                    ),
            new Pose2d(4.9, 0.4, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToDeposit);
  }

  public Command goAndPrepareToDepositFromTag3(){ 
    Trajectory trajectoryToDeposit = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(

                  new Translation2d(1, 0.4)
                    ),
            new Pose2d(3.1, 0.4, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToDeposit);
  }

  /** Tags 1 & 6 */
  public Command goAndPrepareToDepositFromTag1_6(){ 
    Trajectory trajectoryToDeposit = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(

                  new Translation2d(1, -0.4)
                    ),
            new Pose2d(3.1 , -0.4, Rotation2d.fromDegrees(0)), 
            AutoUtility.autoTrajectoryConfig);
    return AutoUtility.trajectoryCommandMaker(CHASSIS, trajectoryToDeposit);
  }
}

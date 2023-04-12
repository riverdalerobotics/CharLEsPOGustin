// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//TODO: wire raspberry pi (no code necessary we are just streaming)
package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDropCubeDoNothing = "Drop Cube Do Nothing";
  private static final String kTag1_6IntakeCone = "Tag 1 and 6 high cube, intake one cone";
  private static final String kTag3_8IntakeCone = "Tag 3 and 8 high cube, intake one cone";
  private static final String kTag3TwoPiece = "Tag 3 High Cube pick up one cone and score it";
  private static final String kTag6TwoPiece = "Tag 6 High Cube pick up one cone and score it";
  private static final String kTag2_7Balance = "Drop loaded cube in mid and dock (Tag 2 and 7)";
  private static final String kTag1_6IntakeConePrepToScore = "Tag 1 and 6 Intake Cone and Prepare to Score auto";
  private static final String kTag3_8IntakeConePrepToScore = "Tag 3 and 8 Intake Cone and Prepare to Score auto";
 
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_chooser.setDefaultOption("Tag 1 and 6 Intake Cone auto", kTag1_6IntakeCone);
    m_chooser.addOption("Tag 3 and 8 Intake Cone auto", kTag3_8IntakeCone);
    m_chooser.addOption("Tag 1 and 6 Intake Cone and Prepare to Score auto", kTag1_6IntakeConePrepToScore);
    m_chooser.addOption("Tag 3 and 8 Intake Cone and Prepare to Score auto", kTag3_8IntakeConePrepToScore);
    m_chooser.addOption("Tag 3 two piece auto", kTag3TwoPiece);
    m_chooser.addOption("Tag 6 two piece auto", kTag6TwoPiece);
    m_chooser.addOption("Tag 2 and 7 Mid Drop Cube Balance", kTag2_7Balance);
    m_chooser.addOption("Drop and finish", kDropCubeDoNothing);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override 
  public void disabledPeriodic() {}


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    /* */
    switch (m_autoSelected) {
      case kDropCubeDoNothing:
        m_autonomousCommand = m_robotContainer.getDropCubeDoNothing();
        break;

      case kTag2_7Balance:
        m_autonomousCommand = m_robotContainer.getMid2_7BalanceAuto();
        break;

      case kTag1_6IntakeCone:
        m_autonomousCommand = m_robotContainer.getTag1_6IntakeAuto();
        break;  
        
      case kTag3_8IntakeCone:
        m_autonomousCommand = m_robotContainer.getTag3_8IntakeAuto();
        break;

      case kTag1_6IntakeConePrepToScore:
        m_autonomousCommand = m_robotContainer.getTag1_6IntakePrepToScoreAuto();
        break;

      case kTag6TwoPiece:
        m_autonomousCommand = m_robotContainer.getTag6TwoPieceAuto();
        break;

      case kTag3TwoPiece:
        m_autonomousCommand = m_robotContainer.getTag3TwoPieceAuto();
        break;

      default:
        // Put default auto code here
        m_autonomousCommand = m_robotContainer.getDropCubeDoNothing();
        break;
    
      
    }
    //TODO: maybe change this back
    //m_autonomousCommand = m_robotContainer.getTag3TwoPieceAuto();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("Successfully scheduled");
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

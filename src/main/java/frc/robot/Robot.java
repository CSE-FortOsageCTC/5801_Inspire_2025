// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.cert.Extension;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Orchestra;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Orchestra orchestra = new Orchestra();
  List<String> music = new ArrayList<String>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // CanBridge.runTCP();

    m_robotContainer = new RobotContainer();

    RobotModeTriggers.autonomous().whileTrue(m_robotContainer.autoChooser.selectedCommandScheduler());

    addAllInstruments();

    // music.add("midis/jeopardy.chrp");
    // music.add("midis/freeBird.chrp");
    // music.add("midis/marioOne.chrp");
    // music.add("midis/marioTwo.chrp");
    // music.add("midis/pirate.chrp");
    // music.add("midis/star.chrp");
    // music.add("midis/guilesTheme.chrp");
    // music.add("midis/TTFATF.chrp");
    // music.add("midis/Sans.chrp");
    // music.add("midis/doom.chrp");


    


    // int index = (int)(Math.random() * music.size());
    // orchestra.loadMusic(music.get(index));

    // playOrchestra();
  }
 

  
//   public void playOrchestra() {
//     int index = (int)(Math.random() * music.size());
//     orchestra.loadMusic(music.get(index));
//     orchestra.play();
// }

// public void stopOrchestra() {
//     orchestra.stop();
// }

// public void restartOrchestra() {
//     if (!orchestra.isPlaying()) {
//         int index = (int)(Math.random() * music.size());
//         orchestra.loadMusic(music.get(index));
//         orchestra.play();
//     }
// }

public void addAllInstruments(){
  Swerve.getInstance().addInstruments(orchestra);
  ExtensionSubsystem.getInstance().addInstruments(orchestra);
  PivotSubsystem.getInstance().addInstruments(orchestra);
  ManipulatorSubsystem.getInstance().addInstruments(orchestra);
  IntakeSubsystem.getInstance().addInstruments(orchestra);

}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // stopOrchestra();
  }

  @Override
  public void disabledPeriodic() {
    // restartOrchestra();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //stopOrchestra();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    PivotSubsystem.getInstance().resetPID();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // playOrchestra();
    //stopOrchestra();

    IntakeSubsystem.getInstance().setIntakeSpeed(0, 0);

    PivotSubsystem.getInstance().resetPID();
    ExtensionSubsystem.getInstance().resetPID();

    LimeLightSubsystem.getLeftInstance().setPipeline(0);
    LimeLightSubsystem.getRightInstance().setPipeline(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

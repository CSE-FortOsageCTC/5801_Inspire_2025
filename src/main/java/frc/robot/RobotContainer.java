// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int throttle = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
//private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton elevatorUpButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton elevatorDownButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton wristUpButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton wristDownButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton wheelForwardButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton wheelBackwardButton = new JoystickButton(driver, XboxController.Button.kA.value);

  /* Operator Buttons */
  //private final JoystickButton moveToL1 = new JoystickButton(operator, XboxController.Button.kA.value);
  //private final JoystickButton moveToL2 = new JoystickButton(operator, XboxController.Button.kX.value);
  //private final JoystickButton moveToL3 = new JoystickButton(operator, XboxController.Button.kB.value);
  //private final JoystickButton moveToL4 = new JoystickButton(operator, XboxController.Button.kY.value);

  /* Subsystems */
  // private final Swerve s_Swerve = new Swerve();
  private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
  private final ManipulatorSubsystem manipulatorSubsystem = ManipulatorSubsystem.getInstance();


  /*commands */
  private final ElevatorDefault elevatorDefaultCommand = new ElevatorDefault(operator);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    // s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, throttle));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    elevatorUpButton.whileTrue(new InstantCommand(() -> elevatorSubsystem.setSpeed(0.4)));
    elevatorDownButton.whileTrue(new InstantCommand(() -> elevatorSubsystem.setSpeed(-0.4)));
    elevatorUpButton.onFalse(new InstantCommand(() -> elevatorSubsystem.setSpeed(0)));
    elevatorDownButton.onFalse(new InstantCommand(() -> elevatorSubsystem.setSpeed(0)));
    elevatorSubsystem.setDefaultCommand(elevatorDefaultCommand);

    wristUpButton.whileTrue(new InstantCommand(() -> manipulatorSubsystem.setWristSpeed(0.1)));
    wristDownButton.whileTrue(new InstantCommand(() -> manipulatorSubsystem.setWristSpeed(-0.1)));
    wheelForwardButton.whileTrue(new InstantCommand(() -> manipulatorSubsystem.setWheelSpeed(0.1)));
    wheelBackwardButton.whileTrue(new InstantCommand(() -> manipulatorSubsystem.setWheelSpeed(-0.1)));

    wristUpButton.onFalse(new InstantCommand(() -> manipulatorSubsystem.setWristSpeed(0)));
    wristDownButton.onFalse(new InstantCommand(() -> manipulatorSubsystem.setWristSpeed(0)));
    wheelForwardButton.onFalse(new InstantCommand(() -> manipulatorSubsystem.setWheelSpeed(0)));
    wheelBackwardButton.onFalse(new InstantCommand(() -> manipulatorSubsystem.setWheelSpeed(0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);
    return null;
  }
}

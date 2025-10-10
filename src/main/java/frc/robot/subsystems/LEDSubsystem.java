// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.PWM;

public class LEDSubsystem extends SubsystemBase {

  private final PWM pwm;
  private static LEDSubsystem ledSubsystem;

  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {  
    pwm = new PWM(1);
  }

  /**
   * Sets color of LEDs
   */
  public void setLEDs () {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    // pwm.setPulseTimeMicroseconds(2125);
    pwm.setSpeed(0.99);

    if(alliance.get().equals(Alliance.Blue)){
      pwm.setSpeed(0.87);
    } else { 
      pwm.setSpeed(0.61);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public static LEDSubsystem getInstance() {
    if (ledSubsystem == null) {
        ledSubsystem = new LEDSubsystem();
    }
    return ledSubsystem;
}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}


package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultTeleop;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Swerve s_Swerve;

  private final Joystick driver;
  private final Joystick operator;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve = Swerve.getInstance();
    driver = new Joystick(0);
    operator = new Joystick(1);

    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
     s_Swerve.setDefaultCommand(new DefaultTeleop(driver, operator));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command placeholder = new Command(){};
    return placeholder;
    
  }
}

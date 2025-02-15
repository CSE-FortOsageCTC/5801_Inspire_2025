
package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AlignToApril;
import frc.robot.commands.ArmDefault;
import frc.robot.commands.DefaultTeleop;
import frc.robot.subsystems.ChoreoSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
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
  private ChoreoSubsystem s_choreoSubsystem;
  private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
  private final ManipulatorSubsystem manipulatorSubsystem = ManipulatorSubsystem.getInstance();
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public AutoChooser autoChooser;

  /* DRIVER BUTTONS */
  private final JoystickButton driver_A_Function = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton driver_B_Function = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton driver_X_Function = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton ddriver_Y_Function = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driver_Start_ZeroHeading = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton driver_Back_Function = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton driver_LeftBumper_Function = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton driver_RightBumper_Function = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final ArmDefault armDefaultCommand = new ArmDefault(driver, operator);
  private final POVButton driverLeftDpad = new POVButton(driver, 270);
  private final POVButton driverRightDpad = new POVButton(driver, 90);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve = Swerve.getInstance();
    s_choreoSubsystem = ChoreoSubsystem.getInstance();

    AlignPosition noPos = AlignPosition.NoPos;

    // Configure the trigger bindings
    configureBindings();

    // Create the auto chooser
      autoChooser = new AutoChooser();

        // Add options to the chooser
        autoChooser.addRoutine("Example Routine", s_choreoSubsystem::pickupAndScoreAuto);
        // autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);

        // Put the auto chooser on the dashboard
        SmartDashboard.putData("hi guys", autoChooser);
  }


  private void configureBindings() {
     s_Swerve.setDefaultCommand(new DefaultTeleop(driver, operator));
     elevatorSubsystem.setDefaultCommand(armDefaultCommand);
     // Initialize Driver Button Functions
     driver_Start_ZeroHeading.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
     driverLeftDpad.whileTrue(new AlignToApril(driver, true));
     driverRightDpad.whileTrue(new AlignToApril(driver, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command placeholder = new Command(){};

    // RobotModeTriggers.autonomous().onTrue(s_choreoSubsystem.pickupAndScoreAuto().cmd());

    return autoChooser.selectedCommand();
    
  }
}


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
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.AlignToApril;
import frc.robot.commands.ArmDefault;
import frc.robot.commands.AutoPickupPiece;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DefaultTeleop;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ChoreoSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Swerve s_Swerve;
  private ChoreoSubsystem s_choreoSubsystem;
  private LimeLightSubsystem limelightLeft;
  private LimeLightSubsystem limelightSky;
  private LimeLightSubsystem limelightRight;
  private final ExtensionSubsystem elevatorSubsystem = ExtensionSubsystem.getInstance();
  private final ManipulatorSubsystem manipulatorSubsystem = ManipulatorSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private final LEDSubsystem ledsubsystem = LEDSubsystem.getInstance();
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public AutoChooser autoChooser;

  /* DRIVER BUTTONS */
  private final JoystickButton driver_A_Function = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton driver_B_Function = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton driver_X_Function = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton driver_Y_Function = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driver_Start_ZeroHeading = new JoystickButton(driver,
      XboxController.Button.kStart.value);
  private final JoystickButton driver_Back_Function = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton driver_LeftBumper_Function = new JoystickButton(driver,
      XboxController.Button.kLeftBumper.value);
  private final JoystickButton driver_RightBumper_Function = new JoystickButton(driver,
      XboxController.Button.kRightBumper.value);
  private final ArmDefault armDefaultCommand = new ArmDefault(driver, operator);
  private final POVButton driverLeftDpad = new POVButton(driver, 270);
  private final POVButton driverRightDpad = new POVButton(driver, 90);
  private final POVButton driverUpDpad = new POVButton(driver, 0);
  private final POVButton driverDownDpad = new POVButton(driver, 180);
  private final JoystickButton driverStartButton = new JoystickButton(driver, XboxController.Button.kStart.value);

  private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton operatorLeftStickDown = new JoystickButton(operator,
      XboxController.Button.kLeftStick.value);
  private final JoystickButton operatorRightStickDown = new JoystickButton(operator,
      XboxController.Button.kRightStick.value);

  private final JoystickButton operatorLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton operatorRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final POVButton operatorUpDPad = new POVButton(operator, 0);
  private final POVButton operatorRightDPad = new POVButton(operator, 90);
  private final POVButton operatorDownDPad = new POVButton(operator, 180);
  private final POVButton operatorLeftDPad = new POVButton(operator, 270);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve = Swerve.getInstance();
    s_choreoSubsystem = ChoreoSubsystem.getInstance();
    limelightLeft = LimeLightSubsystem.getInstance(Constants.limelightLeft);
    limelightSky = LimeLightSubsystem.getInstance(Constants.limelightSky);
    limelightRight = LimeLightSubsystem.getInstance(Constants.limelightRight);

    // Configure the trigger bindings
    configureBindings();

    // Create the auto chooser
    autoChooser = new AutoChooser();

    // Add options to the chooser
    autoChooser.addRoutine("onePiece", s_choreoSubsystem::onePieceAuto);
    autoChooser.addRoutine("two piece ij", s_choreoSubsystem::twoPieceIJAutoL2);
    autoChooser.addRoutine("two piece ef", s_choreoSubsystem::twoPieceEFAuto);
    //autoChooser.addRoutine("L2 IJ", s_choreoSubsystem::twoPieceIJAutoL2); //If we need an L2 Auto

    // autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    s_Swerve.setDefaultCommand(new DefaultTeleop(driver, operator));
    manipulatorSubsystem.setDefaultCommand(armDefaultCommand);
    // Initialize Driver Button Functions
    driver_Start_ZeroHeading.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    driver_X_Function.whileTrue(new AlignToApril(AlignPosition.LeftOffset, true));
    driver_B_Function.whileTrue(new AlignToApril(AlignPosition.RightOffset, true));
    driver_Y_Function.whileTrue(new AutoPickupPiece(0));
    driver_A_Function.whileTrue(new AlignToApril(AlignPosition.CenterOffset, true));

    driverStartButton.onTrue(new ClimbCommand(driver));

    operatorLeftBumper.whileTrue(new IntakeCommand(false));
    operatorRightBumper.whileTrue(new IntakeCommand(true));
    operatorA.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.Ground)));
    operatorX.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L3)));
    operatorB.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L2)));
    operatorY.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)));
    operatorLeftDPad.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)));
    operatorRightDPad.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L1)));
    operatorUpDPad.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.HighAlgae)));
    operatorDownDPad.onTrue(new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.LowAlgae)));



    // operatorA.onFalse(new InstantCommand(() ->
    // ArmPosition.setPosition(ArmPosition.Travel)));
    // operatorX.onFalse(new InstantCommand(() ->
    // ArmPosition.setPosition(ArmPosition.Travel)));
    // operatorB.onFalse(new InstantCommand(() ->
    // ArmPosition.setPosition(ArmPosition.Travel)));
    // operatorY.onFalse(new InstantCommand(() ->
    // ArmPosition.setPosition(ArmPosition.Travel)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command placeholder = new Command() {
    };

    // RobotModeTriggers.autonomous().onTrue(s_choreoSubsystem.pickupAndScoreAuto().cmd());

    return autoChooser.selectedCommand();

  }
}

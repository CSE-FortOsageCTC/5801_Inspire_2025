package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ArmDefault extends Command {
    public final Joystick operator;
    private final Joystick driver;
    public ExtensionSubsystem extensionSubsystem;
    public PivotSubsystem pivotSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    // private SlewRateLimiter elevatorManualSlewRateLimiterUp = new SlewRateLimiter(0.05);
    // private SlewRateLimiter elevatorManualSlewRateLimiterDown = new SlewRateLimiter(0.05);

    public ArmDefault(Joystick driver, Joystick operator) {
        this.driver = driver;
        this.operator = operator;
        this.extensionSubsystem = ExtensionSubsystem.getInstance();
        this.pivotSubsystem = PivotSubsystem.getInstance();
        this.manipulatorSubsystem = ManipulatorSubsystem.getInstance();

        addRequirements(extensionSubsystem, manipulatorSubsystem, pivotSubsystem);
    }

    @Override
    public void execute() {

        if (operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > Constants.stickDeadband) {
            extensionSubsystem.setSetpoint(extensionSubsystem.getManualSetpoint() - 0.1);
        } else if (operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > Constants.stickDeadband) {
            extensionSubsystem.setSetpoint(extensionSubsystem.getManualSetpoint() + 0.1);
        }

        if (operator.getRawAxis(XboxController.Axis.kRightX.value) > Constants.stickDeadband) {
            manipulatorSubsystem.setSetpoint(manipulatorSubsystem.getManualSetpoint() - .1);
        } else if (operator.getRawAxis(XboxController.Axis.kRightX.value) < -Constants.stickDeadband) {
            manipulatorSubsystem.setSetpoint(manipulatorSubsystem.getManualSetpoint() + .1);
        }

        if (operator.getRawButton(XboxController.Button.kBack.value)) {
            System.out.println("(" + PivotSubsystem.getPivotEncoder() + ", " + extensionSubsystem.getExtensionEncoder()
                    + ", " + manipulatorSubsystem.getWristEncoder() + ")");
        }

        if (operator.getRawAxis(XboxController.Axis.kLeftY.value) < -Constants.stickDeadband) {
            pivotSubsystem.setSetpoint(pivotSubsystem.getManualSetpoint() - 0.2);
        } else if (operator.getRawAxis(XboxController.Axis.kLeftY.value) > Constants.stickDeadband) {
            pivotSubsystem.setSetpoint(pivotSubsystem.getManualSetpoint() + 0.2);
        }

        // if (PivotSubsystem.getServo() == 0){//(PivotSubsystem.getStartingDelay() >= 250) {
        //     pivotSubsystem.setPosition();
        // }

        pivotSubsystem.setPosition();

        extensionSubsystem.setPosition();

        manipulatorSubsystem.setPosition();

        // if (!ArmPosition.Manual.equals(manipulatorSubsystem.getArmPosition())) {
        // manipulatorSubsystem.setPosition(ArmPosition.Travel);
        // }
    }
}

// }
// }

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ClimbCommand extends Command{
    private Joystick joystick;
    private ExtensionSubsystem extensionSubsystem;
    private PivotSubsystem pivotSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;

    private boolean isIrreversible;

    public ClimbCommand(Joystick joystick){
        extensionSubsystem = ExtensionSubsystem.getInstance();
        pivotSubsystem = PivotSubsystem.getInstance();
        manipulatorSubsystem = ManipulatorSubsystem.getInstance();

        this.joystick = joystick;

        isIrreversible = false;

        addRequirements(extensionSubsystem, pivotSubsystem, manipulatorSubsystem);
    }
    
    @Override
    public void execute(){
        double speed = 0;

        if(joystick.getPOV() == 0 && joystick.getRawButton(XboxController.Button.kStart.value)){
            ArmPosition.setPosition(ArmPosition.Climb1);
        } else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value) && joystick.getRawButton(XboxController.Button.kStart.value)){
            speed = -0.4;
            // isIrreversible = true;
        } else if (joystick.getPOV() == 180 && joystick.getRawButton(XboxController.Button.kStart.value)){
            pivotSubsystem.setIsClimbing();
            ArmPosition.setPosition(ArmPosition.Climb2);
            isIrreversible = true;
        }

        // System.out.println("Speed = " + speed);

        pivotSubsystem.setClimbingClampSpeed(speed);
        pivotSubsystem.setPosition();
    }

    @Override
    public boolean isFinished() {
        return (!isIrreversible && !joystick.getRawButton(XboxController.Button.kStart.value));
    }

    @Override
    public void end(boolean isFinished) {
        pivotSubsystem.setClimbingClampSpeed(0);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefault extends Command{
    public Joystick operator;
    public ElevatorSubsystem elevatorSubsystem;


    public ElevatorDefault(Joystick operator){
        this.operator = operator;
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();



        addRequirements(elevatorSubsystem);
    }
    @Override
    public void execute(){
        if (operator.getRawButton(XboxController.Button.kA.value)){
            //System.out.println("A Button Detected");
            elevatorSubsystem.setPosition(ElevatorPosition.L1);

        } else if (operator.getRawButton(XboxController.Button.kX.value)){
            //System.out.println("X Button Detected");
            elevatorSubsystem.setPosition(ElevatorPosition.L2);

        } else if (operator.getRawButton(XboxController.Button.kB.value)){
            //System.out.println("B Button Detected");
            elevatorSubsystem.setPosition(ElevatorPosition.L3);

        } else if (operator.getRawButton(XboxController.Button.kY.value)){
            //System.out.println("Y Button Detected");
            elevatorSubsystem.setPosition(ElevatorPosition.L4);

        } else {
            
            elevatorSubsystem.setPosition(ElevatorPosition.Travel);
    
        }
    }
}

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ArmDefault extends Command{
    public final Joystick operator;
    private final Joystick driver;
    private boolean isManualElevator = false;
    private boolean isManualWrist = false;
    public ElevatorSubsystem elevatorSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private SlewRateLimiter elevatorManualSlewRateLimiterUp = new SlewRateLimiter(0.05);
    private SlewRateLimiter elevatorManualSlewRateLimiterDown = new SlewRateLimiter(0.05);


    public ArmDefault(Joystick driver, Joystick operator){
        this.driver = driver;
        this.operator = operator;
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.manipulatorSubsystem = ManipulatorSubsystem.getInstance();



        addRequirements(elevatorSubsystem, manipulatorSubsystem);
    }
    @Override
    public void execute(){
        if (operator.getRawButton(XboxController.Button.kA.value)){
            //System.out.println("A Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L1);
            manipulatorSubsystem.setPosition(ArmPosition.L1);
            isManualElevator = false;
            isManualWrist = false;

        } else if (operator.getRawButton(XboxController.Button.kX.value)){
            //System.out.println("X Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L2);
            manipulatorSubsystem.setPosition(ArmPosition.L2);
            isManualElevator = false;
            isManualWrist = false;

        } else if (operator.getRawButton(XboxController.Button.kB.value)){
            //System.out.println("B Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L3);
            manipulatorSubsystem.setPosition(ArmPosition.L3);
            isManualElevator = false;
            isManualWrist = false;

        } else if (operator.getRawButton(XboxController.Button.kY.value)){
            //System.out.println("Y Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L4);
            manipulatorSubsystem.setPosition(ArmPosition.L4);
            isManualElevator = false;
            isManualWrist = false;

        } else {
            
            if (driver.getRawButton(XboxController.Button.kLeftBumper.value)) {
                elevatorSubsystem.setSpeed(elevatorManualSlewRateLimiterDown.calculate(-0.1));
                isManualElevator = true;
            } else if (driver.getRawButton(XboxController.Button.kRightBumper.value)) {
                elevatorSubsystem.setSpeed(elevatorManualSlewRateLimiterUp.calculate(0.1));
                isManualElevator = true;
            } else if (elevatorSubsystem.getArmPosition().equals(ArmPosition.Manual)) {
                elevatorSubsystem.setSpeed(0);
            }

            if (driver.getRawButton(XboxController.Button.kX.value)) {
                manipulatorSubsystem.setWristSpeed(-0.3);
                isManualWrist = true;
            } else if (driver.getRawButton(XboxController.Button.kY.value)) {
                manipulatorSubsystem.setWristSpeed(0.3);
                isManualWrist = true;
            } else if (manipulatorSubsystem.getArmPosition().equals(ArmPosition.Manual)) {
                manipulatorSubsystem.setWristSpeed(0);
            }

            if (!elevatorSubsystem.getArmPosition().equals(ArmPosition.Manual)) {
                elevatorSubsystem.setPosition(ArmPosition.Travel);
            }

            if (!manipulatorSubsystem.getArmPosition().equals(ArmPosition.Manual)) {
                manipulatorSubsystem.setPosition(ArmPosition.Travel);
            }
            
    
        }
    }
}

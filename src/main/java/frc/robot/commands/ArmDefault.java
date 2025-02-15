package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ArmDefault extends Command{
    public final Joystick operator;
    private final Joystick driver;
    private boolean isManualElevator = false;
    private boolean isManualWrist = false;
    private boolean isManualPivot = false;
    public ElevatorSubsystem elevatorSubsystem;
    public PivotSubsystem pivotSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private SlewRateLimiter elevatorManualSlewRateLimiterUp = new SlewRateLimiter(0.05);
    private SlewRateLimiter elevatorManualSlewRateLimiterDown = new SlewRateLimiter(0.05);

    public ArmDefault(Joystick driver, Joystick operator){
        this.driver = driver;
        this.operator = operator;
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.pivotSubsystem = PivotSubsystem.getInstance();
        this.manipulatorSubsystem = ManipulatorSubsystem.getInstance();

        addRequirements(elevatorSubsystem, manipulatorSubsystem, pivotSubsystem);
    }

    @Override
    public void execute(){
        if (operator.getRawButton(XboxController.Button.kA.value)){
            //System.out.println("A Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L1);
            pivotSubsystem.setPosition(ArmPosition.L1);
            manipulatorSubsystem.setPosition(ArmPosition.L1);
            isManualElevator = false;
            isManualWrist = false;

        } else if (operator.getRawButton(XboxController.Button.kX.value)){
            //System.out.println("X Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L2);
            pivotSubsystem.setPosition(ArmPosition.L2);
            manipulatorSubsystem.setPosition(ArmPosition.L2);
            isManualElevator = false;
            isManualWrist = false;

        } else if (operator.getRawButton(XboxController.Button.kB.value)){
            //System.out.println("B Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L3);
            pivotSubsystem.setPosition(ArmPosition.L3);
            manipulatorSubsystem.setPosition(ArmPosition.L3);
            isManualElevator = false;
            isManualWrist = false;

        } else if (operator.getRawButton(XboxController.Button.kY.value)){
            //System.out.println("Y Button Detected");
            elevatorSubsystem.setPosition(ArmPosition.L4);
            pivotSubsystem.setPosition(ArmPosition.L4);
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
                manipulatorSubsystem.setCoralWristSpeed(-0.3);
                isManualWrist = true;
            } else if (driver.getRawButton(XboxController.Button.kY.value)) {
                manipulatorSubsystem.setCoralWristSpeed(0.3);
                isManualWrist = true;
            } else if (ArmPosition.Manual.equals(manipulatorSubsystem.getArmPosition())) {
                manipulatorSubsystem.setCoralWristSpeed(0);
            }

            if (driver.getPOV() == 0) { // up on d-pad
                pivotSubsystem.setSpeed(0.1);
                isManualPivot = true;
            } else if (driver.getPOV() == 180) { // down on d-pad
                pivotSubsystem.setSpeed(-0.1);
                isManualPivot = true;
            } else if (ArmPosition.Manual.equals(manipulatorSubsystem.getArmPosition())) {
                pivotSubsystem.setSpeed(0);
            }

            if (!ArmPosition.Manual.equals(elevatorSubsystem.getArmPosition())) {
                elevatorSubsystem.setPosition(ArmPosition.Travel);
            }

            if (!ArmPosition.Manual.equals(manipulatorSubsystem.getArmPosition())) {
                manipulatorSubsystem.setPosition(ArmPosition.Travel);
            }

            if (!ArmPosition.Manual.equals(pivotSubsystem.getArmPosition())) {
                pivotSubsystem.setPosition(ArmPosition.Travel);
            }
        }

        // if (manipulatorSubsystem.getProximity() >= 0.20 && manipulatorSubsystem.getProximity() <= 0.28 && !operator.getRawButton(XboxController.Button.kLeftBumper.value)){
        //     manipulatorSubsystem.setAlgaeWheelSpeed(0.3);
        // } else if (operator.getRawButton(XboxController.Button.kLeftBumper.value)){
        //     manipulatorSubsystem.setAlgaeWheelSpeed(-0.3);
        // } else {
        //     manipulatorSubsystem.setAlgaeWheelSpeed(0);
        // }

        SmartDashboard.putString("Wrist Position", manipulatorSubsystem.getArmPosition().toString());
    }
}

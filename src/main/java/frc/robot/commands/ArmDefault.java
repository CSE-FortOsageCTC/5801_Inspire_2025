package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ArmDefault extends Command{
    public final Joystick operator;
    private final Joystick driver;
    private boolean isManualElevator = false;
    private boolean isManualWrist = false;
    private boolean isManualPivot = false;
    public ExtensionSubsystem extensionSubsystem;
    public PivotSubsystem pivotSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private SlewRateLimiter elevatorManualSlewRateLimiterUp = new SlewRateLimiter(0.05);
    private SlewRateLimiter elevatorManualSlewRateLimiterDown = new SlewRateLimiter(0.05);

    public ArmDefault(Joystick driver, Joystick operator){
        this.driver = driver;
        this.operator = operator;
        this.extensionSubsystem = ExtensionSubsystem.getInstance();
        this.pivotSubsystem = PivotSubsystem.getInstance();
        this.manipulatorSubsystem = ManipulatorSubsystem.getInstance();

        addRequirements(extensionSubsystem, manipulatorSubsystem, pivotSubsystem);
    }

    @Override
    public void execute(){

        // if (DriverStation.isAutonomous()) {
            
            // if (!ArmPosition.Manual.equals(elevatorSubsystem.getArmPosition())) {
            //     elevatorSubsystem.setPosition(ArmPosition.Travel);
            // }

            // if (!ArmPosition.Manual.equals(manipulatorSubsystem.getArmPosition())) {
            //     manipulatorSubsystem.setPosition(ArmPosition.Travel);
            // }

            // if (!ArmPosition.Manual.equals(pivotSubsystem.getArmPosition())) {
            //     pivotSubsystem.setPosition(ArmPosition.Travel);
            // }

        // } else if (DriverStation.isTeleopEnabled()) {
        //     if (operator.getRawButton(XboxController.Button.kA.value)){
        //         //elevatorSubsystem.setPosition(ArmPosition.L1);
        //         //pivotSubsystem.setPosition(ArmPosition.L1);
        //         manipulatorSubsystem.setPosition(ArmPosition.L1);
        //         isManualElevator = false;
        //         isManualWrist = false;
    
        //     } else if (operator.getRawButton(XboxController.Button.kX.value)){
        //         //elevatorSubsystem.setPosition(ArmPosition.L2);
        //         //pivotSubsystem.setPosition(ArmPosition.L2);
        //         manipulatorSubsystem.setPosition(ArmPosition.L2);
        //         isManualElevator = false;
        //         isManualWrist = false;
    
        //     } else if (operator.getRawButton(XboxController.Button.kB.value)){
        //         //elevatorSubsystem.setPosition(ArmPosition.L3);
        //         //pivotSubsystem.setPosition(ArmPosition.L3);
        //         manipulatorSubsystem.setPosition(ArmPosition.L3);
        //         isManualElevator = false;
        //         isManualWrist = false;
    
        //     } else if (operator.getRawButton(XboxController.Button.kY.value)){
        //         //elevatorSubsystem.setPosition(ArmPosition.L4);
        //         //pivotSubsystem.setPosition(ArmPosition.L4);
        //         manipulatorSubsystem.setPosition(ArmPosition.L4);
        //         isManualElevator = false;
        //         isManualWrist = false;
    
        //     } else {
                
                if (operator.getPOV() == 0) {
                    extensionSubsystem.setSetpoint(extensionSubsystem.getManualSetpoint() - 0.1);
                    isManualElevator = true;
                } else if (operator.getPOV() == 180) {
                    extensionSubsystem.setSetpoint(extensionSubsystem.getManualSetpoint() + 0.1);
                    isManualElevator = true;
                }
    
                if (driver.getRawButton(XboxController.Button.kX.value)) {
                    manipulatorSubsystem.setWristSpeed(-0.05);
                    isManualWrist = true;
                } else if (driver.getRawButton(XboxController.Button.kY.value)) {
                    manipulatorSubsystem.setWristSpeed(0.05);
                    isManualWrist = true;
                } else if (ArmPosition.Manual.equals(ArmPosition.getPosition())) {
                    manipulatorSubsystem.setWristSpeed(0);
                }

                if (operator.getRawButton(XboxController.Button.kBack.value)) {
                    System.out.println("(" + pivotSubsystem.getPivotEncoder() + ", " + extensionSubsystem.getExtensionEncoder() + ", " + manipulatorSubsystem.getWristEncoder() +")");
                }

                if (driver.getRawButton(XboxController.Button.kA.value) && !manipulatorSubsystem.hasPiece()) { // intake
                    manipulatorSubsystem.setIntakeSpeed(-1);
                    isManualWrist = true;
                } else if (driver.getRawButton(XboxController.Button.kB.value)) { // outtake
                    manipulatorSubsystem.setIntakeSpeed(1);
                    isManualWrist = true;
                } else {
                    manipulatorSubsystem.setIntakeSpeed(0);
                }
    
                if (operator.getRawButton(XboxController.Button.kLeftBumper.value)) {
                    pivotSubsystem.setSpeed(-0.1);
                    isManualPivot = true;
                } else if (operator.getRawButton(XboxController.Button.kRightBumper.value)) {
                    pivotSubsystem.setSpeed(0.1);
                    isManualPivot = true;
                } else if (ArmPosition.Manual.equals(ArmPosition.getPosition())) {
                    pivotSubsystem.setSpeed(0);
                }

                extensionSubsystem.setPosition();

                pivotSubsystem.setPosition();
                
                manipulatorSubsystem.setPosition();
    
                // if (!ArmPosition.Manual.equals(manipulatorSubsystem.getArmPosition())) {
                //     manipulatorSubsystem.setPosition(ArmPosition.Travel);
                // }
            }
        }
        
//     }
// }

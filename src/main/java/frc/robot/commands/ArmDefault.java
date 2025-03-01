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
                
                if (operator.getPOV() == 0) {
                    extensionSubsystem.setSetpoint(extensionSubsystem.getManualSetpoint() - 0.1);
                    isManualElevator = true;
                } else if (operator.getPOV() == 180) {
                    extensionSubsystem.setSetpoint(extensionSubsystem.getManualSetpoint() + 0.1);
                    isManualElevator = true;
                }
    
                if (driver.getRawButton(XboxController.Button.kX.value)) {
                    manipulatorSubsystem.setSetpoint(manipulatorSubsystem.getManualSetpoint() - .1);
                    isManualWrist = true;
                } else if (driver.getRawButton(XboxController.Button.kY.value)) {
                    manipulatorSubsystem.setSetpoint(manipulatorSubsystem.getManualSetpoint() + .1);
                    isManualWrist = true;
                }

                if (operator.getRawButton(XboxController.Button.kBack.value)) {
                    System.out.println("(" + PivotSubsystem.getPivotEncoder() + ", " + extensionSubsystem.getExtensionEncoder() + ", " + manipulatorSubsystem.getWristEncoder() +")");
                }
    
                if (operator.getRawButton(XboxController.Button.kLeftBumper.value)) {
                    pivotSubsystem.setSetpoint(pivotSubsystem.getManualSetpoint() - 0.2);
                    isManualPivot = true;
                } else if (operator.getRawButton(XboxController.Button.kRightBumper.value)) {
                    pivotSubsystem.setSetpoint(pivotSubsystem.getManualSetpoint() + 0.2);
                    isManualPivot = true;
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

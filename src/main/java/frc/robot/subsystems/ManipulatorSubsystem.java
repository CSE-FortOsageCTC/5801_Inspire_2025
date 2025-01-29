package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase{

    private static SparkMax intakeWheel;
    private static SparkMax intakeWrist;

    private static ManipulatorSubsystem manipulatorSubsystem;

    private ManipulatorSubsystem() {
        intakeWheel = new SparkMax(56, MotorType.kBrushless);
        intakeWrist = new SparkMax(55, MotorType.kBrushless);
    }

    public static ManipulatorSubsystem getInstance(){
        if (manipulatorSubsystem == null) {
            manipulatorSubsystem = new ManipulatorSubsystem();
        }

        return manipulatorSubsystem;
    }

    public void setWristSpeed(double speed){
        intakeWrist.set(speed);
    }
    
    public void setWheelSpeed(double speed){
        intakeWheel.set(speed);
    }
}

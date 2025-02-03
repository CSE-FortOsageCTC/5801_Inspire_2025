package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

public class ManipulatorSubsystem extends SubsystemBase{

    private static SparkMax intakeWheel;
    private static SparkMax intakeWrist;

    private static ManipulatorSubsystem manipulatorSubsystem;

    private static ArmPosition lastPosition;

    private static ProfiledPIDController pidController;

    private ManipulatorSubsystem() {
        intakeWheel = new SparkMax(56, MotorType.kBrushless);
        intakeWrist = new SparkMax(55, MotorType.kBrushless);

        pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(70, 70));
        pidController.setTolerance(0.1);
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

    public void setPosition(ArmPosition position){
        if (position != lastPosition) {
            pidController.reset(getWristEncoder());
        }
        
        double calculation = MathUtil.clamp(pidController.calculate(getWristEncoder(), position.telescope), -1, 1);
        intakeWrist.set(calculation);
        SmartDashboard.putNumber("PID Output", calculation);
        lastPosition = position;
    }

    private double getWristEncoder(){
        return intakeWrist.getEncoder().getPosition();
    }

    public ArmPosition getArmPosition() {
        return lastPosition;
    }
}

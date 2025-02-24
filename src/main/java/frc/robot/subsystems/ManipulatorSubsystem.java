package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

public class ManipulatorSubsystem extends SubsystemBase{

    private static SparkMax intakeWheel;
    private static TalonFX intakeWrist;

    private static DutyCycleEncoder wristEncoder;

    private static ManipulatorSubsystem manipulatorSubsystem;

    private static ArmPosition lastPosition;

    private static ProfiledPIDController pidController;

    private ManipulatorSubsystem() {
        intakeWheel = new SparkMax(55, MotorType.kBrushless);
        intakeWrist = new TalonFX(56);

        wristEncoder = new DutyCycleEncoder(0);

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
        lastPosition = ArmPosition.Manual;
    }
    
    public void setIntakeSpeed(double speed){
        intakeWheel.set(speed);
    }

    public void setPosition(ArmPosition position){
        if (position != lastPosition) {
            pidController.reset(getWristEncoder());
        }
        
        double calculation = MathUtil.clamp(pidController.calculate(getWristEncoder(), position.extension), -1, 1);
        intakeWrist.set(calculation);
        // SmartDashboard.putNumber("PID Output", calculation);
        lastPosition = position;
    }

    public boolean atPosition() {
        return pidController.atSetpoint();
    }

    public double getWristEncoder(){
        return intakeWrist.getPosition().getValueAsDouble();
    }

    public ArmPosition getArmPosition() {
        return lastPosition;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Kraken Encoder", getWristEncoder());
        SmartDashboard.putNumber("Wrist Absolute Encoder", wristEncoder.get());
        pidController.setP(0);
    }
}

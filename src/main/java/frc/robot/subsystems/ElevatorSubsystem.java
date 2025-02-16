package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ArmPosition;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private static TalonFX elevatorMaster;
    private static TalonFX elevatorFollower;

    private static ProfiledPIDController pidController;

    private static ElevatorSubsystem elevatorSubsystem;

    private static ArmPosition lastElevatorPosition = ArmPosition.Travel;

    public static ElevatorSubsystem getInstance(){
        if(elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    private ElevatorSubsystem(){
        elevatorMaster = new TalonFX(50); //SparkMax(50, MotorType.kBrushless);
        elevatorFollower = new TalonFX(51); //SparkMax(51, MotorType.kBrushless);

        pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(200, 75));
        pidController.setTolerance(0.1);
    }

    public void setPosition(ArmPosition position){
        if (position != lastElevatorPosition) {
            pidController.reset(getElevatorEncoder());
        }
        
        double calculation = MathUtil.clamp(pidController.calculate(getElevatorEncoder(), position.telescope), -1, 1);
        elevatorMaster.set(calculation);
        SmartDashboard.putNumber("PID Output", calculation);
        lastElevatorPosition = position;
    }

    public void setSpeed(double speed){
        elevatorMaster.set(speed);
        lastElevatorPosition = ArmPosition.Manual;
    }

    private double getElevatorEncoder(){
        return elevatorFollower.getPosition().getValueAsDouble();
    }

    public ArmPosition getArmPosition() {
        return lastElevatorPosition;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
        SmartDashboard.putNumber("SparkMax Speed fr", elevatorFollower.get());
        pidController.setPID(0.075, 0, 0);
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorPosition;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private static SparkMax elevatorMaster;
    private static SparkMax elevatorFollower;

    private static ElevatorSubsystem elevatorSubsystem;

    private ElevatorSubsystem(){
        elevatorMaster = new SparkMax(50, MotorType.kBrushless);
        elevatorFollower = new SparkMax(51, MotorType.kBrushless);
    }



    public void setPosition(ElevatorPosition position){
        
    }

    private double getElevatorEncoder(){
        return elevatorFollower.getEncoder().getPosition();
    }

    public static ElevatorSubsystem getInstance(){
        if(elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
    }
}

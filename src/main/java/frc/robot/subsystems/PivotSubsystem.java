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


public class PivotSubsystem extends SubsystemBase{
    private static TalonFX pivotMaster;
    private static TalonFX pivotFollower;

    private static ProfiledPIDController pidController;

    private static PivotSubsystem pivotSubsystem;

    private static ArmPosition lastPivotPosition = ArmPosition.Travel;

    public static PivotSubsystem getInstance(){
        if(pivotSubsystem == null) {
            pivotSubsystem = new PivotSubsystem();
        }
        return pivotSubsystem;
    }

    private PivotSubsystem(){
        pivotMaster = new TalonFX(50); //SparkMax(50, MotorType.kBrushless);
        pivotFollower = new TalonFX(51); //SparkMax(51, MotorType.kBrushless);

        pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(200, 75));
        pidController.setTolerance(0.1);
    }

    public void setPosition(ArmPosition position){
        if (position != lastPivotPosition) {
            pidController.reset(getPivotEncoder());
        }
        
        double calculation = MathUtil.clamp(pidController.calculate(getPivotEncoder(), position.telescope), -1, 1);
        pivotMaster.set(calculation);
        SmartDashboard.putNumber("PID Output", calculation);
        lastPivotPosition = position;
    }

    public void setSpeed(double speed){
        pivotMaster.set(speed);
        lastPivotPosition = ArmPosition.Manual;
    }

    private double getPivotEncoder(){
        return pivotFollower.getPosition().getValueAsDouble();
    }

    public ArmPosition getArmPosition() {
        return lastPivotPosition;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());
        pidController.setPID(0.075, 0, 0);
    }

}

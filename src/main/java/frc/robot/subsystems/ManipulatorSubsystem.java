package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

public class ManipulatorSubsystem extends SubsystemBase{

    private static SparkMax intakeCoralWheel;
    private static SparkMax intakeCoralWrist;
    private static SparkMax intakeAlgaeWheel;

    private Canandcolor canandcolor;

    private static ManipulatorSubsystem manipulatorSubsystem;

    private static ArmPosition lastPosition;

    private static ProfiledPIDController pidController;

    private CanandcolorSettings settings = new CanandcolorSettings();

    private ManipulatorSubsystem() {
        intakeCoralWheel = new SparkMax(56, MotorType.kBrushless);
        intakeCoralWrist = new SparkMax(55, MotorType.kBrushless);

        intakeAlgaeWheel = new SparkMax(60, MotorType.kBrushless);

        pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(70, 70));
        pidController.setTolerance(0.1);

        canandcolor = new Canandcolor(30);

        settings.setLampLEDBrightness(0.1);
        
        canandcolor.setSettings(settings);
    }

    public static ManipulatorSubsystem getInstance(){
        if (manipulatorSubsystem == null) {
            manipulatorSubsystem = new ManipulatorSubsystem();
        }

        return manipulatorSubsystem;
    }

    public void setCoralWristSpeed(double speed){
        intakeCoralWrist.set(speed);
        lastPosition = ArmPosition.Manual;
    }
    
    public void setCoralWheelSpeed(double speed){
        intakeCoralWheel.set(speed);
    }

    public void setAlgaeWheelSpeed(double speed){
        intakeAlgaeWheel.set(speed);
    }

    public void setPosition(ArmPosition position){
        if (position != lastPosition) {
            pidController.reset(getWristEncoder());
        }
        
        double calculation = MathUtil.clamp(pidController.calculate(getWristEncoder(), position.telescope), -1, 1);
        intakeCoralWrist.set(calculation);
        SmartDashboard.putNumber("PID Output", calculation);
        lastPosition = position;
    }

    private double getWristEncoder(){
        return intakeCoralWrist.getEncoder().getPosition();
    }

    public ArmPosition getArmPosition() {
        return lastPosition;
    }

    public double getProximity() {
        return canandcolor.getProximity();
    }

    public Double getHSVHue() {
        return canandcolor.getHSVHue();
    }

    public Double getHSVSaturation() {
        return canandcolor.getHSVSaturation();
    }

    public Double getHSVValue() {
        return canandcolor.getHSVValue();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("CAC Proximity", getProximity());
        SmartDashboard.putNumber("CAC Hue", getHSVHue());
        SmartDashboard.putNumber("CAC Saturation", getHSVSaturation());
        SmartDashboard.putNumber("CAC Value", getHSVValue());

        pidController.setP(0.2);
    }
}

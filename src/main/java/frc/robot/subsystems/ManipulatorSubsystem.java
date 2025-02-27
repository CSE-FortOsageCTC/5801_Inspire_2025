package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;

public class ManipulatorSubsystem extends SubsystemBase{

    private static SparkMax intakeWheel;
    private static TalonFX intakeWrist;

    //private static DutyCycleEncoder wristEncoder;

    // private static Canandcolor canandcolor;
    // private CanandcolorSettings cacSettings;

    private static ManipulatorSubsystem manipulatorSubsystem;

    private static ArmPosition lastPosition;

    private static ProfiledPIDController pidController;

    private ManipulatorSubsystem() {
        intakeWheel = new SparkMax(55, MotorType.kBrushless);
        intakeWrist = new TalonFX(56);

        intakeWrist.setNeutralMode(NeutralModeValue.Brake);

        intakeWrist.setPosition(0);

        pidController = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(100, 75));
        pidController.setTolerance(0.1);

        // canandcolor = new Canandcolor(30);
        // cacSettings = new CanandcolorSettings();

        // cacSettings.setLampLEDBrightness(0.2);
        
        // canandcolor.setSettings(cacSettings);
    }

    public static ManipulatorSubsystem getInstance(){
        if (manipulatorSubsystem == null) {
            manipulatorSubsystem = new ManipulatorSubsystem();
        }

        return manipulatorSubsystem;
    }

    public void setWristSpeed(double speed){
        intakeWrist.set(speed);
        ArmPosition.setPosition(ArmPosition.Manual);
        lastPosition = ArmPosition.Manual;
    }
    
    public void setIntakeSpeed(double speed){
        intakeWheel.set(speed);
    }

    public void setPosition(){
        if (ArmPosition.getPosition() != lastPosition) {
            pidController.reset(getWristEncoder());
        }

        double setpoint = ArmPosition.getPosition().manipulator;

        if (setpoint == -1) {
            return;
        }
        
        double calculation = MathUtil.clamp(pidController.calculate(getWristEncoder(), setpoint), -1, 1);
        intakeWrist.set(calculation);
        // SmartDashboard.putNumber("PID Output", calculation);
        lastPosition = ArmPosition.getPosition();
    }

    private boolean atLimit(boolean positive){
        double encoder = getWristEncoder();
        return (ExtensionSubsystem.isExtended() && (!positive && encoder >= Constants.wristUpperLimitExtended) || (positive && encoder <= Constants.wristUpperLimitExtended)) 
           || (!ExtensionSubsystem.isExtended() && (!positive && encoder >= Constants.wristUpperLimitRetracted) || (positive && encoder <= Constants.wristUpperLimitRetracted));
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

    // public double getProximity() {
    //     return canandcolor.getProximity();
    // }

    // public boolean hasPiece() {
    //     return canandcolor.getProximity() < 0.1;
    // }

    // public Double getHSVHue() {
    //     return canandcolor.getHSVHue();
    // }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Wrist Kraken Encoder", getWristEncoder());
        //SmartDashboard.putNumber("Wrist Absolute Encoder", wristEncoder.get());
        //SmartDashboard.putNumber("HSV CanAndColor", getHSVHue());
        //pidController.setP(0.3);

        //SmartDashboard.putNumber("Proximity", getProximity());
    }
}

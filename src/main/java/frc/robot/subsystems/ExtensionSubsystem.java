package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
    private static TalonFX extensionMaster;
    private static TalonFX extensionFollower;

    private static ProfiledPIDController pidController;

    private static ExtensionSubsystem extensionSubsystem;

    private static ArmPosition lastExtensionPosition = ArmPosition.Travel;

    private double manualSetpoint;

    public static ExtensionSubsystem getInstance() {
        if (extensionSubsystem == null) {
            extensionSubsystem = new ExtensionSubsystem();
        }
        return extensionSubsystem;
    }

    private ExtensionSubsystem() {
        extensionMaster = new TalonFX(50); // SparkMax(50, MotorType.kBrushless);
        extensionFollower = new TalonFX(51); // SparkMax(51, MotorType.kBrushless);

        extensionMaster.setNeutralMode(NeutralModeValue.Brake);
        extensionFollower.setNeutralMode(NeutralModeValue.Brake);

        extensionMaster.setPosition(0);

        //extensionMaster.setControl(new MusicTone())

        extensionFollower.setControl(new Follower(extensionMaster.getDeviceID(), true));
        ;
        
        // TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        // currentLimitsConfigs.withStatorCurrentLimit(20);
        // currentLimitsConfigs.withStatorCurrentLimitEnable(true);

        // extensionConfig.withCurrentLimits(currentLimitsConfigs);

        // extensionMaster.getConfigurator().apply(extensionConfig);
        // extensionFollower.getConfigurator().apply(extensionConfig);


        manualSetpoint = getExtensionEncoder();

        pidController = new ProfiledPIDController(0.25, 0, 0, new TrapezoidProfile.Constraints(200, 100));
        pidController.setTolerance(0.1);
    }

    public void setPosition() {
        if (ArmPosition.getPosition() != lastExtensionPosition) {
            pidController.reset(getExtensionEncoder());
        }

        double setpoint = ArmPosition.getPosition().extension;

        if (setpoint == -1) {
            setpoint = manualSetpoint;
        } else {
            manualSetpoint = getExtensionEncoder();
        }

        if (!PivotSubsystem.nearSetpoint() && ArmPosition.getPosition().extension != -1) {
            setpoint = 0;
        }

        // if (PivotSubsystem.getPivotEncoder() > Constants.pivotExtensionLimit) {
        //     manualSetpoint = 0;
        // }
        double pivotDegrees = MathUtil.clamp((PivotSubsystem.getPivotEncoder() * -Constants.pivotDegreesPerEncoder), 0, 89);
        SmartDashboard.putNumber("Pivot Degrees", pivotDegrees);
        double maxHypotenuse = Constants.Swerve.XLimit/Math.cos(Units.degreesToRadians(pivotDegrees))-40;
        double maxSetpoint = MathUtil.clamp(-maxHypotenuse/Constants.extensionInchesPerEncoder, Constants.extensionLowerLimit, Constants.extensionUpperLimit);
        SmartDashboard.putNumber("Max Extension Setpoint", maxSetpoint);
        if(setpoint < maxSetpoint){
            manualSetpoint = maxSetpoint;
            setpoint = maxSetpoint;
        }

        double calculation = MathUtil.clamp(pidController.calculate(getExtensionEncoder(), setpoint), -1, 1);
        privSetSpeed(calculation);
        SmartDashboard.putNumber("PID Output", calculation);
        lastExtensionPosition = ArmPosition.getPosition();
    }

    public static boolean atPosition() {
        return pidController.atSetpoint();
    }

    public static boolean isExtended() {
        return extensionSubsystem.getExtensionEncoder() < -5.0;
    }

    public static double getMotorSpeed() {
        return extensionMaster.get();
    }

    public static boolean nearSetpoint() {
        double encoder = extensionSubsystem.getExtensionEncoder();
        return Math.abs(encoder - ArmPosition.getPosition().extension) <= 2.5;
    }

    public void setSetpoint(double setpoint) {
        ArmPosition.setPosition(ArmPosition.Manual);
        lastExtensionPosition = ArmPosition.Manual;
        manualSetpoint = MathUtil.clamp(setpoint, Constants.extensionLowerLimit, Constants.extensionUpperLimit);
        setPosition();
    }

    private void privSetSpeed(double speed) {

        boolean isPositive = speed > 0;

        if (atLimit(isPositive)) {
            speed = 0;
        } // else if (nearLimit(isPositive)){
          // speed = 0.05;
          // }

        //System.out.println("extension speed: " + speed);
        extensionMaster.set(speed);
    }

    private boolean atLimit(boolean goingDown) {
        double encoder = getExtensionEncoder();
        return (goingDown && encoder >= Constants.extensionUpperLimit)
                || (!goingDown && encoder <= Constants.extensionLowerLimit);
    }

    // private boolean nearLimit(boolean positive) {
    // double encoder = getExtensionEncoder();
    // return (positive && Constants.extensionUpperLimit - encoder >= 10) ||
    // (!positive && Constants.extensionLowerLimit - encoder >= 10);
    // }

    public double getManualSetpoint() {
        return manualSetpoint;
    }

    public double getExtensionEncoder() {
        return extensionMaster.getPosition().getValueAsDouble();
    }

    public ArmPosition getArmPosition() {
        return lastExtensionPosition;
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Extension Motor Speed", getMotorSpeed());
        SmartDashboard.putNumber("Extension Encoder", getExtensionEncoder());
        // SmartDashboard.putNumber("Extension Manual Setpoint", manualSetpoint);
        // pidController.setPID(0.25, 0, 0);
    }
}

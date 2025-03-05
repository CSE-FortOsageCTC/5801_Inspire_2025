package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class ManipulatorSubsystem extends SubsystemBase {

    private static TalonFX intakeWrist;

    // private static DutyCycleEncoder wristEncoder;

    private static ManipulatorSubsystem manipulatorSubsystem;

    private static ExtensionSubsystem extensionSubsystem;

    private static ArmPosition lastPosition;

    private double manualSetpoint;

    private double setpoint;

    private static ProfiledPIDController pidController;

    private ManipulatorSubsystem() {

        intakeWrist = new TalonFX(56);

        intakeWrist.setNeutralMode(NeutralModeValue.Brake);

        intakeWrist.setPosition(0);
        
        // TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        // currentLimitsConfigs.withStatorCurrentLimit(20);
        // currentLimitsConfigs.withStatorCurrentLimitEnable(true);

        // wristConfig.withCurrentLimits(currentLimitsConfigs);

        // intakeWrist.getConfigurator().apply(wristConfig);

        extensionSubsystem = ExtensionSubsystem.getInstance();

        pidController = new ProfiledPIDController(0.225, 0, 0, new TrapezoidProfile.Constraints(100, 75));
        pidController.setTolerance(0.1);
    }

    public static ManipulatorSubsystem getInstance() {
        if (manipulatorSubsystem == null) {
            manipulatorSubsystem = new ManipulatorSubsystem();
        }

        return manipulatorSubsystem;
    }

    public void setSetpoint(double setpoint) {
        ArmPosition.setPosition(ArmPosition.Manual);
        lastPosition = ArmPosition.Manual;
        manualSetpoint = MathUtil.clamp(setpoint, Constants.wristLowerLimit,
                ExtensionSubsystem.isExtended() ? Constants.wristUpperLimitExtended
                        : Constants.wristUpperLimitRetracted);
        //setPosition();
    }

    private void privSetSpeed(double speed) {

        boolean isPositive = speed > 0;

        if (atLimit(isPositive)) {
            speed = 0;
        }

        intakeWrist.set(speed);
    }

    public void setPosition() {
        if (ArmPosition.getPosition() != lastPosition) {
            pidController.reset(getWristEncoder());
        }

        setpoint = ArmPosition.getPosition().manipulator;

        if (setpoint == -1) {
            setpoint = manualSetpoint;
        } else {
            manualSetpoint = getWristEncoder();
        }

        boolean isDown = extensionSubsystem.getExtensionEncoder() - ArmPosition.getPosition().extension >= 0;

        if (!ExtensionSubsystem.nearSetpoint() && isDown) {
            setpoint = ArmPosition.getPosition().manipulator;
        } else if (!ExtensionSubsystem.nearSetpoint() && !isDown && ArmPosition.getPosition().manipulator == -1) {
            setpoint = manualSetpoint;
        }

        if (!ExtensionSubsystem.isExtended() && getWristEncoder() > Constants.wristUpperLimitRetracted) {
            manualSetpoint = Constants.wristUpperLimitRetracted - 1;
        }

        double calculation = MathUtil.clamp(pidController.calculate(getWristEncoder(), setpoint), -1, 1);
        privSetSpeed(calculation);
        // SmartDashboard.putNumber("PID Output", calculation);
        lastPosition = ArmPosition.getPosition();
    }

    private boolean atLimit(boolean positive) {
        double encoder = getWristEncoder();
        return (positive && encoder >= (ExtensionSubsystem.isExtended() ? Constants.wristUpperLimitExtended
                : Constants.wristUpperLimitRetracted))
                || (!positive && encoder <= Constants.wristLowerLimit);
    }

    public static boolean nearSetpoint() {
        double encoder = manipulatorSubsystem.getWristEncoder();
        return Math.abs(encoder - ArmPosition.getPosition().manipulator) <= 1;
    }

    public static boolean atPosition() {
        return pidController.atSetpoint();
    }

    public double getWristEncoder() {
        return intakeWrist.getPosition().getValueAsDouble();
    }

    public ArmPosition getArmPosition() {
        return lastPosition;
    }

    public double getManualSetpoint() {
        return manualSetpoint;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Wrist Kraken Encoder", getWristEncoder());
        // SmartDashboard.putNumber("Wrist Absolute Encoder", wristEncoder.get());
        // SmartDashboard.putNumber("HSV CanAndColor", getHSVHue());
        // pidController.setP(0.3);

        // SmartDashboard.putNumber("Proximity", getProximity());
    }
}

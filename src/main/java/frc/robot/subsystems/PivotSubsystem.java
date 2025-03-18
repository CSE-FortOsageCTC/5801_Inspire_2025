package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.AlignPosition;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private static TalonFX pivotMaster;
    private static TalonFX pivotFollower;

    private static Servo servo;

    private static DutyCycleEncoder pivotEncoder;

    private static ProfiledPIDController pidController;

    private static PivotSubsystem pivotSubsystem;

    private static ExtensionSubsystem extensionSubsystem;

    private static double setpoint;

    private double manualSetpoint;

    private boolean isClimbing;

    private static int startingDelay = 0;

    private static ArmPosition lastPivotPosition = ArmPosition.Travel;

    

    private SparkMax climbingClamp;
    private SparkMaxConfig config;

    public static PivotSubsystem getInstance() {
        if (pivotSubsystem == null) {
            pivotSubsystem = new PivotSubsystem();
        }
        return pivotSubsystem;
    }

    private PivotSubsystem() {
        pivotMaster = new TalonFX(52); // SparkMax(50, MotorType.kBrushless);
        pivotFollower = new TalonFX(53); // SparkMax(51, MotorType.kBrushless);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        // currentLimitsConfigs.withSupplyCurrentLimit(120);
        // currentLimitsConfigs.withSupplyCurrentLimitEnable(true);

        pivotConfig.withCurrentLimits(currentLimitsConfigs);

        // pivotMaster.getConfigurator().apply(pivotConfig);
        // pivotFollower.getConfigurator().apply(pivotConfig);

        pivotMaster.setNeutralMode(NeutralModeValue.Brake);
        pivotFollower.setNeutralMode(NeutralModeValue.Brake);

        servo = new Servo(0);

        extensionSubsystem = ExtensionSubsystem.getInstance();

        pivotFollower.setControl(new Follower(pivotMaster.getDeviceID(), true));

        pivotEncoder = new DutyCycleEncoder(1);

        System.out.println(pivotEncoder.get());

        AlignPosition noPos = AlignPosition.NoPos; //Do not remove! The robot will break.

        climbingClamp = new SparkMax(20, MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
        config.idleMode(IdleMode.kCoast);

        climbingClamp.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        manualSetpoint = getPivotEncoder();

        System.out.println(pivotEncoder.get());

        isClimbing = false;

        pivotMaster.setPosition(pivotEncoder.get() * -53.8);

        pidController = new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(150, 100));
        pidController.setTolerance(0.1);
    }

    public static void resetStartDelay() {
        startingDelay = 0;
    }

    public static int getStartingDelay() {
        return startingDelay;
    }
    public static double getServo(){
        return servo.get();
    }

    private void privSetSpeed(double speed) {
        boolean isPositive = speed > 0;
        if (atLimit(isPositive)) {
            speed = 0;
        }

        pivotMaster.set(speed);
    }

    public void setSetpoint(double setpoint) {
        ArmPosition.setPosition(ArmPosition.Manual);
        lastPivotPosition = ArmPosition.Manual;
        manualSetpoint = MathUtil.clamp(setpoint, Constants.pivotLowerLimit, Constants.pivotUpperLimit);
        //setPosition();
    }

    private boolean atLimit(boolean positive) {
        double encoder = getPivotEncoder();
        return (positive && encoder >= Constants.pivotUpperLimit)
                || (!positive && encoder <= Constants.pivotLowerLimit);
    }

    public static boolean nearSetpoint() {
        double encoder = getPivotEncoder();
        return Math.abs(encoder - ArmPosition.getPosition().pivot) <= 15;
    }

    public void setPosition() {
        if (ArmPosition.getPosition() != lastPivotPosition) {
            pidController.reset(getPivotEncoder());
        }

        setpoint = ArmPosition.getPosition().pivot;

        if (ArmPosition.getPosition().pivot == -1) {
            setpoint = manualSetpoint;
        } else {
            manualSetpoint = setpoint;
        }

        boolean isGoingDown = extensionSubsystem.getExtensionEncoder() - ArmPosition.getPosition().extension >= 0;
        if (!ExtensionSubsystem.atPosition() && !isGoingDown) {
            setpoint = getPivotEncoder();
            manualSetpoint = getPivotEncoder();
        }

        if ((!ExtensionSubsystem.atPosition() && isGoingDown) || ExtensionSubsystem.atPosition()) {

            double calculation = MathUtil.clamp(pidController.calculate(getPivotEncoder(), setpoint), -1, 1);
            privSetSpeed(calculation);
            // SmartDashboard.putNumber("PID Output", calculation);
            lastPivotPosition = ArmPosition.getPosition();
        } else {
            setpoint = getPivotEncoder();
            privSetSpeed(0);
        }
    }

    public double getManualSetpoint() {
        return manualSetpoint;
    }

    public static boolean atPosition() {
        return pidController.atSetpoint();
    }

    public void setSpeed(double speed) {
        privSetSpeed(speed);
        ArmPosition.setPosition(ArmPosition.Manual);
        lastPivotPosition = ArmPosition.Manual;
    }

    public static double getPivotEncoder() {
        return pivotMaster.getPosition().getValueAsDouble();
    }

    public ArmPosition getArmPosition() {
        return lastPivotPosition;
    }

    public void setIsClimbing(){
        isClimbing = true;
    }

    public void setClimbingClampSpeed(double speed){
        // if (climbingClamp.getBusVoltage() > 10){
        //     speed = 0;
        // }

        climbingClamp.set(speed);

        SmartDashboard.putNumber("Climb Clamp Voltage", climbingClamp.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Kraken Encoder", getPivotEncoder());
        SmartDashboard.putNumber("Pivot Absolute Encoder", pivotEncoder.get());
        // pidController.setPID(0.3, 0, 0);
        if (DriverStation.isDisabled()) {
            resetStartDelay();
        }
        if (startingDelay < 250) {
            startingDelay++;
        }

        servo.set(0);//isClimbing ? 1 : 0);
    }
}

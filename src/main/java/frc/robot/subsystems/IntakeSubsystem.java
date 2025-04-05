package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.GrappleJNI;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

public class IntakeSubsystem extends SubsystemBase {

    private static SparkMax coralIntake;
    private static TalonFX algaeIntake;

    private static IntakeSubsystem intakeSubsystem;

    private static LaserCan laserCan;
    
    private boolean hasPiece = true;

    private LimeLightSubsystem limelightRight;
    private LimeLightSubsystem limelightLeft;

    // private static Canandcolor canandcolor;
    // private CanandcolorSettings cacSettings;

    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    private IntakeSubsystem() {
        coralIntake = new SparkMax(54, MotorType.kBrushless);
        algaeIntake = new TalonFX(55);

        limelightRight = LimeLightSubsystem.getRightInstance();
        limelightLeft = LimeLightSubsystem.getLeftInstance();

        SparkMaxConfig coralConfig = new SparkMaxConfig();

        coralConfig.smartCurrentLimit(20, 20);

        //algaeConfig.follow(54, false);

        coralIntake.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // algaeIntake.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        laserCan = new LaserCan(30);

        try {
            laserCan.setRangingMode(RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
        // canandcolor = new Canandcolor(30);
        // cacSettings = new CanandcolorSettings();

        // cacSettings.setLampLEDBrightness(0.2);

        // canandcolor.setSettings(cacSettings);
    }

    public void setIntakeSpeed(double coralSpeed, double algaeSpeed) {
        if (hasPiece() && coralSpeed < 0 && ArmPosition.Ground.equals(ArmPosition.getPosition())) {
            coralSpeed = 0;
            algaeSpeed = 0;
        }
        coralIntake.set(coralSpeed);
        algaeIntake.set(algaeSpeed);
    }

    public double getProximity() {
        Measurement proximity = laserCan.getMeasurement();
        if (proximity == null) {
            return 0;
        }
        return proximity.distance_mm;
        // return canandcolor.getProximity();
    }

    public boolean hasPiece() {
        // return canandcolor.getProximity() == 0? false:canandcolor.getProximity() < 0.07;
        return hasPiece;
    }

    // public Double getHSVHue() {
    //     // return canandcolor.getHSVHue();
    //     return 0.0;
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Proximity", getProximity());
        if (!hasPiece && getProximity() < 50 && (ArmPosition.Ground.equals(ArmPosition.getPosition()) || ArmPosition.HumanP.equals(ArmPosition.getPosition())) && PivotSubsystem.nearSetpoint() && ManipulatorSubsystem.nearSetpoint()) {
            hasPiece = true;
            if (DriverStation.isAutonomous()) {
                limelightLeft.setPipeline(0);
                limelightRight.setPipeline(0);
            }
        } else if (hasPiece && getProximity() > 50) {
            hasPiece = false;
            if (DriverStation.isAutonomous()) {
                limelightLeft.setPipeline(2);
                limelightRight.setPipeline(2);
            }
            System.out.println("We changed pipelines to 2 (Coral Detection)");
        }
        
    }
}

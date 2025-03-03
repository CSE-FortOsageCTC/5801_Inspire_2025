package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private static SparkMax intakeWheel;

    private static IntakeSubsystem intakeSubsystem;

    private static Canandcolor canandcolor;
    private CanandcolorSettings cacSettings;

    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    private IntakeSubsystem() {
        intakeWheel = new SparkMax(55, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.smartCurrentLimit(20, 20);
        intakeWheel.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        canandcolor = new Canandcolor(30);
        cacSettings = new CanandcolorSettings();

        cacSettings.setLampLEDBrightness(0.2);

        canandcolor.setSettings(cacSettings);
    }

    public void setIntakeSpeed(double speed) {
        if (hasPiece() && speed < 0) {
            speed = 0;
        }
        intakeWheel.set(speed);
    }

    public double getProximity() {
        return canandcolor.getProximity();
    }

    public boolean hasPiece() {
        return canandcolor.getProximity() == 0? false:canandcolor.getProximity() < 0.07;
    }

    public Double getHSVHue() {
        return canandcolor.getHSVHue();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("intake current", intakeWheel.getOutputCurrent());
        SmartDashboard.putNumber("intake proximity", canandcolor.getProximity());
    }
}

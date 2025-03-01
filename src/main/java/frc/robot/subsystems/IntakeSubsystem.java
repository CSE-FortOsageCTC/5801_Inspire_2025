package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
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

        canandcolor = new Canandcolor(30);
        cacSettings = new CanandcolorSettings();

        cacSettings.setLampLEDBrightness(0.2);
        
        canandcolor.setSettings(cacSettings);
    }

    public void setIntakeSpeed(double speed){
        intakeWheel.set(speed);
    }

    public double getProximity() {
        return canandcolor.getProximity();
    }

    public boolean hasPiece() {
        return canandcolor.getProximity() < 0.1;
    }

    public Double getHSVHue() {
        return canandcolor.getHSVHue();
    }

}

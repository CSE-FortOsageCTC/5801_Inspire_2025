package frc.robot.subsystems;

import java.util.ArrayList;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CanAndColorSubsystem extends SubsystemBase{
    
    private Canandcolor canandcolor;

    private static CanAndColorSubsystem canAndColorSubsystem;
    private CanandcolorSettings settings = new CanandcolorSettings();

    public static CanAndColorSubsystem getInstance() {
        if (canAndColorSubsystem == null) {
            canAndColorSubsystem = new CanAndColorSubsystem();
        }
        return canAndColorSubsystem;
    }

    private CanAndColorSubsystem() {
        // CanandEventLoop.getInstance();
        canandcolor = new Canandcolor(30);

        settings.setLampLEDBrightness(0.1);
        
        canandcolor.setSettings(settings);

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
    }
}
